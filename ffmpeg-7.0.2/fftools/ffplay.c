/*
 * Copyright (c) 2003 Fabrice Bellard
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * simple media player based on the FFmpeg libraries
 */

#include "config.h"
#include "config_components.h"
#include <inttypes.h>
#include <math.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/eval.h"
#include "libavutil/mathematics.h"
#include "libavutil/pixdesc.h"
#include "libavutil/imgutils.h"
#include "libavutil/dict.h"
#include "libavutil/fifo.h"
#include "libavutil/parseutils.h"
#include "libavutil/samplefmt.h"
#include "libavutil/time.h"
#include "libavutil/bprint.h"
#include "libavformat/avformat.h"
#include "libavdevice/avdevice.h"
#include "libswscale/swscale.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "libswresample/swresample.h"

#include "libavfilter/avfilter.h"
#include "libavfilter/buffersink.h"
#include "libavfilter/buffersrc.h"

#include <SDL.h>
#include <SDL_thread.h>

#include "cmdutils.h"
#include "ffplay_renderer.h"
#include "opt_common.h"

const char program_name[] = "ffplay";
const int program_birth_year = 2003;

#define MAX_QUEUE_SIZE (15 * 1024 * 1024)
#define MIN_FRAMES 25
#define EXTERNAL_CLOCK_MIN_FRAMES 2
#define EXTERNAL_CLOCK_MAX_FRAMES 10

/* Minimum SDL audio buffer size, in samples. */
#define SDL_AUDIO_MIN_BUFFER_SIZE 512
/* Calculate actual buffer size keeping in mind not cause too frequent audio callbacks */
#define SDL_AUDIO_MAX_CALLBACKS_PER_SEC 30

/* Step size for volume control in dB */
#define SDL_VOLUME_STEP (0.75)

/* no AV sync correction is done if below the minimum AV sync threshold */
#define AV_SYNC_THRESHOLD_MIN 0.04
/* AV sync correction is done if above the maximum AV sync threshold */
#define AV_SYNC_THRESHOLD_MAX 0.1
/* If a frame duration is longer than this, it will not be duplicated to compensate AV sync */
#define AV_SYNC_FRAMEDUP_THRESHOLD 0.1
/* no AV correction is done if too big error */
#define AV_NOSYNC_THRESHOLD 10.0

/* maximum audio speed change to get correct sync */
#define SAMPLE_CORRECTION_PERCENT_MAX 10

/* external clock speed adjustment constants for realtime sources based on buffer fullness */
#define EXTERNAL_CLOCK_SPEED_MIN  0.900
#define EXTERNAL_CLOCK_SPEED_MAX  1.010
#define EXTERNAL_CLOCK_SPEED_STEP 0.001

/* we use about AUDIO_DIFF_AVG_NB A-V differences to make the average */
//* 用于计算音视频差异（A-V differences）的平均值，使用的样本数量约为 AUDIO_DIFF_AVG_NB（即 20 个）。
#define AUDIO_DIFF_AVG_NB   20

/* polls for possible required screen refresh at least this often, should be less than 1/fps */
#define REFRESH_RATE 0.01    //* 100 fps,  1s / frame_number

/* NOTE: the size must be big enough to compensate the hardware audio buffersize size */
/* TODO: We assume that a decoded and resampled frame fits into this buffer */
#define SAMPLE_ARRAY_SIZE (8 * 65536)

#define CURSOR_HIDE_DELAY 1000000

#define USE_ONEPASS_SUBTITLE_RENDER 1

typedef struct MyAVPacketList {
    AVPacket *pkt;
    int serial;
} MyAVPacketList;

typedef struct PacketQueue {
    AVFifo *pkt_list;   // AVFifo 是 FFmpeg 提供的一个通用 FIFO 实现，支持动态扩容和线程安全操作。
    int nb_packets;     // 当前队列中的数据包数量。
    int size;           // 当前队列中所有数据包的总大小（以字节为单位）。
    int64_t duration;   // 当前队列中所有数据包的总时长（以时间基为单位）。
    int abort_request;  // 队列是否已中止。。通常用于在退出程序或 Seek 操作时清空队列。
    int serial;         // 队列的序列号，用于标识队列的状态。在 Seek 操作时，序列号会递增，以区分旧数据和新数据。
    SDL_mutex *mutex;   
    SDL_cond *cond;
} PacketQueue;

#define VIDEO_PICTURE_QUEUE_SIZE 3
#define SUBPICTURE_QUEUE_SIZE 16
#define SAMPLE_QUEUE_SIZE 9
#define FRAME_QUEUE_SIZE FFMAX(SAMPLE_QUEUE_SIZE, FFMAX(VIDEO_PICTURE_QUEUE_SIZE, SUBPICTURE_QUEUE_SIZE))

typedef struct AudioParams {
    int freq;
    AVChannelLayout ch_layout;
    enum AVSampleFormat fmt;
    int frame_size;
    int bytes_per_sec;
} AudioParams;

//* 管理音视频同步的时钟数据结构。它主要用于记录和计算播放时间（PTS，Presentation Time Stamp），并处理播放速度、暂停状态以及时钟的序列号等信息。
typedef struct Clock {
    double pts;           /* clock base */  //前时钟的时间点，通常是从音视频帧中提取的 PTS 值。它是音视频同步的核心依据，用于确定当前应该播放哪一帧。
    double pts_drift;     /* clock base minus time at which we updated the clock */ // 用于计算时钟的漂移（drift），即时钟时间与实际系统时间的偏差。通过这个值，可以调整时钟的同步状态。
    double last_updated;    // pts_drift = pts - last_updated.   时钟最后一次更新的时间
    double speed;           // 时钟的播放速度，通常为 1.0（正常速度）。速度大于 1.0，表示快进；如果小于 1.0，表示慢放.  这个值会影响 pts 的计算。
    int serial;           /* clock is based on a packet with this serial */  //是一个标识符，用于标记时钟的当前状态。当音视频流发生跳转（seek）或重置时，serial 会递增，以区分新旧时钟状态。通过比较 serial，可以检测时钟是否已经过时（obsolete）。
    int paused;     // 时钟的暂停标志 1，表示时钟已暂停，时间不会更新。 0，表示时钟正在运行。
    int *queue_serial;    /* pointer to the current packet queue serial, used for obsolete clock detection */
            // 指向与当前时钟关联的数据包队列的序列号。
            // 通过比较 serial 和* queue_serial，可以检测时钟是否已经过时。
            // 如果 serial 与* queue_serial 不匹配，说明时钟已经失效，需要重新同步。
} Clock;

typedef struct FrameData {
    int64_t pkt_pos;
} FrameData;

/* Common struct for handling all types of decoded data and allocated render buffers. */
//* 用于处理解码后的音视频数据以及分配的渲染缓冲区。
// 统一管理解码后的音视频帧和字幕帧。
// 存储帧的元数据（如时间戳、宽高、格式等）。
// 提供帧的状态信息（如是否已上传到 GPU）。
// 支持多线程或异步解码场景中的帧顺序管理。
typedef struct Frame {
    AVFrame *frame; // 存储解码后的音视频帧数据。对于视频帧，它包含像素数据（如 YUV 或 RGB）；对于音频帧，它包含音频采样数据。
    AVSubtitle sub; // 储解码后的字幕数据。如果当前帧包含字幕，则会将字幕信息存储在这里。
    int serial;     // 帧的序列号，用于标识帧的顺序。在多线程或异步解码的场景中，序列号可以帮助确保帧的正确顺序。
    double pts;           /* presentation timestamp for the frame */ // 显示时间戳（Presentation Timestamp），表示帧应该在何时被显示或播放。
    double duration;      /* estimated duration of the frame */     // 帧的持续时间，表示帧应该显示的时长。
    int64_t pos;          /* byte position of the frame in the input file */    // 帧在输入文件中的字节位置。用于定位帧在文件中的位置，便于 Seek 操作或调试。
    int width;      // 帧的宽度（以像素为单位）。仅对视频帧有效。
    int height;     // 帧的高度（以像素为单位）。仅对视频帧有效。
    int format;     //帧的格式。对于视频帧，表示像素格式（如 AV_PIX_FMT_YUV420P）；对于音频帧，表示采样格式（如 AV_SAMPLE_FMT_FLTP）。
    AVRational sar; // 帧的样本宽高比（Sample Aspect Ratio）。用于调整视频帧的显示比例。
    int uploaded;   // 标志位，表示帧是否已经上传到 GPU 或渲染器。通常用于硬件加速渲染的场景。
    int flip_v;     // 标志位，表示是否需要垂直翻转帧。某些视频格式可能需要翻转才能正确显示。
} Frame;
// 为什么需要将字幕信息放在 Frame 结构中？
// 同步播放的需求：
//  字幕通常需要与视频帧或音频帧同步显示。例如，某个字幕需要在特定的视频帧或音频时间点显示。
//  将字幕信息与视频帧、音频帧一起存储在 Frame 结构体中，可以方便地根据 pts（显示时间戳）进行同步。
// 统一管理：
//  在多媒体播放器中，视频、音频和字幕帧通常会被放入同一个队列中，按照 pts 排序后依次渲染。
//  将字幕信息放在 Frame 结构体中，可以避免为字幕单独设计一个队列，简化了代码逻辑。
// 减少上下文切换：
//  如果字幕信息单独存储，那么在渲染时需要额外处理字幕与音视频帧的同步问题，增加了复杂性。
//  将字幕信息与音视频帧放在同一个结构中，可以减少上下文切换，提高代码的可读性和可维护性。

//* 管理帧（Frame）的队列，通常用于多媒体播放器中，用于存储解码后的视频帧、音频帧和字幕帧。
// 它通过环形缓冲区（Circular Buffer）的方式实现，支持多线程环境下的生产者和消费者模式。
typedef struct FrameQueue
{
    Frame queue[FRAME_QUEUE_SIZE];  // 存储帧的环形缓冲区。FRAME_QUEUE_SIZE 是队列的最大容量，通常定义为常量。
    int rindex;     // 读索引（Read Index），表示当前可以读取的帧的位置。
    int windex;     // 写索引（Write Index），表示当前可以写入的帧的位置。
    int size;       // 当前队列中的帧数量。
    int max_size;   // 队列的最大容量（即 FRAME_QUEUE_SIZE）
    int keep_last;  // 标志位，表示是否保留最后一帧。通常用于视频播放器中，确保在 Seek 操作后仍然可以显示最后一帧。
    int rindex_shown;   // 标志位，表示当前帧是否已经显示过。用于避免重复渲染。
    SDL_mutex *mutex;   // 用于保护队列的线程安全。在多线程环境下，确保同时只有一个线程访问队列。
    SDL_cond *cond;
    PacketQueue *pktq;  // ：指向关联的数据包队列（PacketQueue）。通常用于在帧队列和数据包队列之间建立联系，例如在帧队列为空时，从数据包队列中获取新的数据包进行解码。
} FrameQueue;

enum {
    AV_SYNC_AUDIO_MASTER, /* default choice */
    AV_SYNC_VIDEO_MASTER,
    AV_SYNC_EXTERNAL_CLOCK, /* synchronize to an external clock */
};

typedef struct Decoder {
    AVPacket *pkt;
    PacketQueue *queue;
    AVCodecContext *avctx;
    int pkt_serial;
    int finished;
    int packet_pending;
    SDL_cond *empty_queue_cond;
    int64_t start_pts;
    AVRational start_pts_tb;
    int64_t next_pts;
    AVRational next_pts_tb;
    SDL_Thread *decoder_tid;
} Decoder;

typedef struct VideoState {
    SDL_Thread *read_tid;
    const AVInputFormat *iformat;
    int abort_request;
    int force_refresh;
    int paused; // 播放器是否处于暂停状态。
    int last_paused;
    int queue_attachments_req; // 是否需要处理附加图片的请求标志。
    int seek_req; // 是否有一个寻址请求需要处理. 
        //当用户拖动进度条或调用跳转函数时，会将 seek_req 设置为 1，并设置相关的寻址参数（如目标时间戳 seek_pos 和相对偏移 seek_rel）。
    int seek_flags; // 寻址标志，例如 AVSEEK_FLAG_BACKWARD（向后寻址）或 AVSEEK_FLAG_BYTE（按字节寻址）
            // #define AVSEEK_FLAG_BACKWARD 1 ///< seek backward
            // #define AVSEEK_FLAG_BYTE     2 ///< seeking based on position in bytes
            // #define AVSEEK_FLAG_ANY      4 ///< seek to any frame, even non-keyframes
            // #define AVSEEK_FLAG_FRAME    8 ///< seeking based on frame number
    int64_t seek_pos;   // 用户指定的目标时间戳（通常是秒或微秒）。
    int64_t seek_rel;   // 相对偏移量，用于调整寻址范围。
    int read_pause_return;  // 处理暂停时的返回值。
    AVFormatContext *ic;
    int realtime;

    Clock audclk;
    Clock vidclk;
    Clock extclk;

    FrameQueue pictq;
    FrameQueue subpq;
    FrameQueue sampq;

    Decoder auddec;
    Decoder viddec;
    Decoder subdec;

    int audio_stream;               // 当前音频流的索引。

    int av_sync_type;

    double audio_clock;             // 当前音频时钟的时间。
    int audio_clock_serial;         // 音频时钟的序列号，用于检测时钟是否发生变化。
    double audio_diff_cum; /* used for AV difference average computation */
    double audio_diff_avg_coef; //* 差值的平均系数，用于平滑差值
    double audio_diff_threshold;
    int audio_diff_avg_count;
    AVStream *audio_st;
    PacketQueue audioq;
    int audio_hw_buf_size;
    uint8_t *audio_buf;
    uint8_t *audio_buf1;
    unsigned int audio_buf_size; /* in bytes */ //* 当前音频缓冲区的大小。
    unsigned int audio_buf1_size;
    int audio_buf_index; /* in bytes */ //* 当前音频缓冲区的读取位置。
    int audio_write_buf_size;       // 剩余未写入的音频数据大小
    int audio_volume;               // 音频音量。
    int muted;                      // 静音标志。
    struct AudioParams audio_src;   // 描述 原始音频数据的格式，即从解码器输出的音频数据的格式。
    struct AudioParams audio_filter_src; // 描述 音频过滤器链的输入格式，即经过初步处理后的音频数据格式
    struct AudioParams audio_tgt;    // : 描述 音频目标格式，即最终输出到音频设备的音频数据格式。
    struct SwrContext *swr_ctx;     // 音频重采样上下文
    int frame_drops_early; // 记录在视频播放过程中提前丢弃的视频帧数量。尤其是在视频帧的显示时间早于主时钟时，播放器主动丢弃了多少帧。
    int frame_drops_late;   // 记录延迟丢弃的视频帧数量。
        // 在视频播放器中，帧丢弃（Frame Dropping）是一种常见的优化策略，用于处理以下情况：
        //     视频帧的显示时间早于主时钟：如果视频帧的显示时间戳（PTS）比当前的主时钟时间早，说明这帧已经“过时”了，播放它会导致画面延迟或不同步。
        //     系统性能不足：如果解码或渲染速度跟不上视频的帧率，丢弃一些帧可以避免播放卡顿。
        // 帧丢弃的目的是确保视频播放的流畅性和同步性，但过度丢弃帧可能会导致画面不连贯或跳帧。

    // 早或晚是针对渲染过程说的，但都是在解码之后，解码之前一般不丢帧，因为可能导致gop中后续参考帧解码失败。但如果连续丢弃gop中后部所有帧，则可以在解码前丢帧

    enum ShowMode {
        SHOW_MODE_NONE = -1, SHOW_MODE_VIDEO = 0, SHOW_MODE_WAVES, SHOW_MODE_RDFT, SHOW_MODE_NB
    } show_mode;
    int16_t sample_array[SAMPLE_ARRAY_SIZE];
    int sample_array_index;
    int last_i_start;
    AVTXContext *rdft;
    av_tx_fn rdft_fn;
    int rdft_bits;
    float *real_data;
    AVComplexFloat *rdft_data;
    int xpos;
    double last_vis_time; // 上次刷新时间
    SDL_Texture *vis_texture;
    SDL_Texture *sub_texture;
    SDL_Texture *vid_texture;

    int subtitle_stream;
    AVStream *subtitle_st;
    PacketQueue subtitleq;

    double frame_timer; // 记录视频帧的显示时间。
    double frame_last_returned_time; // 记录上一次从滤镜图（Filter Graph）获取视频帧的时间。  在每次从滤镜图的输出端（filt_out）获取视频帧之前，记录当前时间。
    double frame_last_filter_delay; // 记录上一次滤镜处理的延迟时间。 在每次从滤镜图的输出端（filt_out）获取视频帧之后，计算滤镜处理的延迟。
    int video_stream;
    AVStream *video_st;
    PacketQueue videoq;
    double max_frame_duration;      // maximum duration of a frame - above this, we consider the jump a timestamp discontinuity
    struct SwsContext *sub_convert_ctx;
    int eof;                        // 如果为 1，表示已经到达文件末尾；如果为 0，表示尚未到达文件末尾。

    char *filename;                 // 输入文件路径
    int width, height, xleft, ytop; // 宽，高，左起始，顶起始
    int step;   // 是否需要跳转到下一帧。

    int vfilter_idx;    // 滤镜索引
    AVFilterContext *in_video_filter;   // the first filter in the video chain
    AVFilterContext *out_video_filter;  // the last filter in the video chain
    AVFilterContext *in_audio_filter;   // the first filter in the audio chain
    AVFilterContext *out_audio_filter;  // the last filter in the audio chain
    AVFilterGraph* agraph;              // audio filter graph
    // 音频过滤器图（is->agraph）是 FFmpeg 中用于对音频帧进行处理的工具。它可以实现多种音频处理操作，如重采样、声道转换、音量调整等。
    // 音频过滤器图用于对音频帧进行必要的处理，以确保音频帧的格式、声道布局和采样率与播放设备的要求一致。例如：
    // 如果音频帧的采样率与播放设备的采样率不一致，可以通过重采样过滤器进行调整。
    // 如果音频帧的声道布局与播放设备的声道布局不一致，可以通过声道转换过滤器进行调整。

    int last_video_stream, last_audio_stream, last_subtitle_stream;

    SDL_cond *continue_read_thread;
} VideoState;

/* options specified by the user */
static const AVInputFormat *file_iformat;
static const char *input_filename;
static const char *window_title;
static int default_width  = 640;
static int default_height = 480;
static int screen_width  = 0;
static int screen_height = 0;
static int screen_left = SDL_WINDOWPOS_CENTERED;
static int screen_top = SDL_WINDOWPOS_CENTERED;
static int audio_disable;
static int video_disable;
static int subtitle_disable;
static const char* wanted_stream_spec[AVMEDIA_TYPE_NB] = {0};
static int seek_by_bytes = -1;
static float seek_interval = 10;
static int display_disable;
static int borderless;
static int alwaysontop;
static int startup_volume = 100;
static int show_status = -1;
static int av_sync_type = AV_SYNC_AUDIO_MASTER;
static int64_t start_time = AV_NOPTS_VALUE; // 开始播放时间 
static int64_t duration = AV_NOPTS_VALUE; // 播放时长
static int fast = 0;
static int genpts = 0;
static int lowres = 0;
static int decoder_reorder_pts = -1;
static int autoexit;
static int exit_on_keydown;
static int exit_on_mousedown;
static int loop = 1; // 循环播放次数，1表示只播放一次； 0，表示无限循环播放。
static int framedrop = -1;
static int infinite_buffer = -1; //* 是否启用无限缓冲区
static enum ShowMode show_mode = SHOW_MODE_NONE;
static const char *audio_codec_name;
static const char *subtitle_codec_name;
static const char *video_codec_name;
double rdftspeed = 0.02;
static int64_t cursor_last_shown;
static int cursor_hidden = 0;
static const char **vfilters_list = NULL; // *存储用户指定的视频滤镜链。例如：用户可以通过命令行参数指定一组视频滤镜（如缩放、旋转、裁剪等）。 -vf "scale=640:480,rotate=90"
static int nb_vfilters = 0;
static char *afilters = NULL;
static int autorotate = 1;
static int find_stream_info = 1;
static int filter_nbthreads = 0;
static int enable_vulkan = 0;
static char *vulkan_params = NULL;
static const char *hwaccel = NULL; //* 解码硬件加速

/* current context */
static int is_full_screen;
static int64_t audio_callback_time;

#define FF_QUIT_EVENT    (SDL_USEREVENT + 2)

static SDL_Window *window;
static SDL_Renderer *renderer;
static SDL_RendererInfo renderer_info = {0};
static SDL_AudioDeviceID audio_dev;

static VkRenderer *vk_renderer;

static const struct TextureFormatEntry {
    enum AVPixelFormat format;
    int texture_fmt;
} sdl_texture_format_map[] = { // 一个映射表，将 SDL 纹理格式（texture_fmt）映射到 FFmpeg 像素格式（format）。
    { AV_PIX_FMT_RGB8,           SDL_PIXELFORMAT_RGB332 },
    { AV_PIX_FMT_RGB444,         SDL_PIXELFORMAT_RGB444 },
    { AV_PIX_FMT_RGB555,         SDL_PIXELFORMAT_RGB555 },
    { AV_PIX_FMT_BGR555,         SDL_PIXELFORMAT_BGR555 },
    { AV_PIX_FMT_RGB565,         SDL_PIXELFORMAT_RGB565 },
    { AV_PIX_FMT_BGR565,         SDL_PIXELFORMAT_BGR565 },
    { AV_PIX_FMT_RGB24,          SDL_PIXELFORMAT_RGB24 },
    { AV_PIX_FMT_BGR24,          SDL_PIXELFORMAT_BGR24 },
    { AV_PIX_FMT_0RGB32,         SDL_PIXELFORMAT_RGB888 },
    { AV_PIX_FMT_0BGR32,         SDL_PIXELFORMAT_BGR888 },
    { AV_PIX_FMT_NE(RGB0, 0BGR), SDL_PIXELFORMAT_RGBX8888 },
    { AV_PIX_FMT_NE(BGR0, 0RGB), SDL_PIXELFORMAT_BGRX8888 },
    { AV_PIX_FMT_RGB32,          SDL_PIXELFORMAT_ARGB8888 },
    { AV_PIX_FMT_RGB32_1,        SDL_PIXELFORMAT_RGBA8888 },
    { AV_PIX_FMT_BGR32,          SDL_PIXELFORMAT_ABGR8888 },
    { AV_PIX_FMT_BGR32_1,        SDL_PIXELFORMAT_BGRA8888 },
    { AV_PIX_FMT_YUV420P,        SDL_PIXELFORMAT_IYUV },
    { AV_PIX_FMT_YUYV422,        SDL_PIXELFORMAT_YUY2 },
    { AV_PIX_FMT_UYVY422,        SDL_PIXELFORMAT_UYVY },
    { AV_PIX_FMT_NONE,           SDL_PIXELFORMAT_UNKNOWN },
};

static int opt_add_vfilter(void *optctx, const char *opt, const char *arg)
{
    int ret = GROW_ARRAY(vfilters_list, nb_vfilters);
    if (ret < 0)
        return ret;

    vfilters_list[nb_vfilters - 1] = av_strdup(arg);
    if (!vfilters_list[nb_vfilters - 1])
        return AVERROR(ENOMEM);

    return 0;
}

static inline
int cmp_audio_fmts(enum AVSampleFormat fmt1, int64_t channel_count1,
                   enum AVSampleFormat fmt2, int64_t channel_count2)
{
    /* If channel count == 1, planar and non-planar formats are the same */
    if (channel_count1 == 1 && channel_count2 == 1)
        return av_get_packed_sample_fmt(fmt1) != av_get_packed_sample_fmt(fmt2);
    else
        return channel_count1 != channel_count2 || fmt1 != fmt2;
}

//* 是 packet_queue_put 的实际实现，负责将数据包添加到队列中，并更新队列的状态。
static int packet_queue_put_private(PacketQueue* q, AVPacket* pkt)
{
    MyAVPacketList pkt1;
    int ret;

    if (q->abort_request)
       return -1;

    //* 将传入的 AVPacket 封装到一个 MyAVPacketList 结构体中。
    pkt1.pkt = pkt;
    pkt1.serial = q->serial;

    //* 将封装后的数据包（pkt1）写入队列的 FIFO 缓冲区（q->pkt_list）。
    ret = av_fifo_write(q->pkt_list, &pkt1, 1);
    if (ret < 0)
        return ret; // 写入失败（ret < 0），返回错误码。
    // 更新队列的状态： 
    q->nb_packets++; // 更新队列中的数据包数量。
    q->size += pkt1.pkt->size + sizeof(pkt1); // 更新队列的总大小。 pkt1.pkt->size 是数据包的大小。 sizeof(pkt1) 是 MyAVPacketList 结构体的大小。
    q->duration += pkt1.pkt->duration; // 更新队列的总时长。
    /* XXX: should duplicate packet data in DV case */ //* 在DV情况下是否应该重复数据包数据
    SDL_CondSignal(q->cond); // 通知等待的线程（如解码线程）有新数据包可用
    return 0;
}

//* 将一个 AVPacket 数据包放入指定的 PacketQueue 队列中。
static int packet_queue_put(PacketQueue *q, AVPacket *pkt)
{
    AVPacket *pkt1;
    int ret;

    //* 放入队列之前，会创建一个新的 AVPacket 对象，并将输入数据包的内容移动到新对象中。
    // 创建新的packet对象来接受移动进来的数据入队，保证数据仅被队列持有，而不被其他外部共享
    pkt1 = av_packet_alloc();
    if (!pkt1) {
        // 如果分配失败，释放输入数据包（av_packet_unref(pkt)）并返回错误（-1）。
        av_packet_unref(pkt);
        return -1;
    }
    // 将输入数据包（pkt）的内容移动到新分配的数据包（pkt1）中。
    av_packet_move_ref(pkt1, pkt); // 移动引用后，pkt 会被置为空，而 pkt1 将持有原始数据包的内容。

    SDL_LockMutex(q->mutex);
    ret = packet_queue_put_private(q, pkt1); // 是实际的入队操作函数。
    SDL_UnlockMutex(q->mutex);

    //* 入队失败（ret < 0），释放新分配的数据包（av_packet_free(&pkt1)）。
    if (ret < 0)
        av_packet_free(&pkt1);

    return ret;
}

static int packet_queue_put_nullpacket(PacketQueue *q, AVPacket *pkt, int stream_index)
{
    pkt->stream_index = stream_index;
    return packet_queue_put(q, pkt);
}

/* packet queue handling */
static int packet_queue_init(PacketQueue *q)
{
    memset(q, 0, sizeof(PacketQueue));
    q->pkt_list = av_fifo_alloc2(1, sizeof(MyAVPacketList), AV_FIFO_FLAG_AUTO_GROW);
    if (!q->pkt_list)
        return AVERROR(ENOMEM);
    q->mutex = SDL_CreateMutex();
    if (!q->mutex) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    q->cond = SDL_CreateCond();
    if (!q->cond) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    q->abort_request = 1;
    return 0;
}

static void packet_queue_flush(PacketQueue *q)
{
    MyAVPacketList pkt1;

    SDL_LockMutex(q->mutex);
    while (av_fifo_read(q->pkt_list, &pkt1, 1) >= 0)
        av_packet_free(&pkt1.pkt);
    q->nb_packets = 0;
    q->size = 0;
    q->duration = 0;
    q->serial++;
    SDL_UnlockMutex(q->mutex);
}

static void packet_queue_destroy(PacketQueue *q)
{
    packet_queue_flush(q);
    av_fifo_freep2(&q->pkt_list);
    SDL_DestroyMutex(q->mutex);
    SDL_DestroyCond(q->cond);
}

static void packet_queue_abort(PacketQueue *q)
{
    SDL_LockMutex(q->mutex);

    q->abort_request = 1;

    SDL_CondSignal(q->cond);

    SDL_UnlockMutex(q->mutex);
}

static void packet_queue_start(PacketQueue *q)
{
    SDL_LockMutex(q->mutex);
    q->abort_request = 0;
    q->serial++;
    SDL_UnlockMutex(q->mutex);
}

/* return < 0 if aborted, 0 if no packet and > 0 if packet.  */
//* 数据包队列（PacketQueue）的获取操作，用于从队列中取出一个 AVPacket
// q：指向 PacketQueue 的指针，表示数据包队列。
// pkt：用于存储获取到的数据包。
// block：是否以阻塞模式等待数据包。
//     1：阻塞模式，如果队列为空，则等待直到有数据包。
//     0：非阻塞模式，如果队列为空，立即返回。
// serial：用于返回数据包的序列号（可选）。
// > 0：成功获取到一个数据包。
// 0：队列为空，且 block 参数为 0（非阻塞模式）。
// < 0：队列被中止（abort_request 为 1）。
static int packet_queue_get(PacketQueue* q, AVPacket* pkt, int block, int* serial)
{
    MyAVPacketList pkt1;
    int ret;

    SDL_LockMutex(q->mutex);

    for (;;) {
        //* 检查队列是否终止
        if (q->abort_request) {
            ret = -1;
            break;
        }

        //* 尝试从队列中取数据包
        if (av_fifo_read(q->pkt_list, &pkt1, 1) >= 0) {
            q->nb_packets--;
            q->size -= pkt1.pkt->size + sizeof(pkt1);
            q->duration -= pkt1.pkt->duration;
            av_packet_move_ref(pkt, pkt1.pkt);//  将数据包从队列中移动到 pkt，避免额外的内存拷贝。
            if (serial)
                *serial = pkt1.serial;
            av_packet_free(&pkt1.pkt);
            ret = 1;
            break;
        } else if (!block) { // 非阻塞时读取失败意味队列为空
            ret = 0;
            break;
        } else { // 阻塞时读取失败，在这里等待新数据包填充
            SDL_CondWait(q->cond, q->mutex);
        }
    }
    SDL_UnlockMutex(q->mutex);
    return ret;
}

static int decoder_init(Decoder *d, AVCodecContext *avctx, PacketQueue *queue, SDL_cond *empty_queue_cond) {
    memset(d, 0, sizeof(Decoder));
    d->pkt = av_packet_alloc();
    if (!d->pkt)
        return AVERROR(ENOMEM);
    d->avctx = avctx;
    d->queue = queue;
    d->empty_queue_cond = empty_queue_cond;
    d->start_pts = AV_NOPTS_VALUE;
    d->pkt_serial = -1;
    return 0;
}

//* 从解码器中获取解码后的音视频帧或字幕帧。
// frame：用于存储解码后的音视频帧。
// sub：用于存储解码后的字幕帧。
// 成功获取帧返回 1。解码结束返回 0。出错情况返回负值。
static int decoder_decode_frame(Decoder *d, AVFrame *frame, AVSubtitle *sub) {
    int ret = AVERROR(EAGAIN); // AVERROR(EAGAIN)，表示暂时没有可用的帧。

    //* 直到解码一帧/解码结束/发生错误返回
    for (;;) { 
        //* 尝试解码, 当前队列序列号与解码器序列号需要一致，确保数据同步
        if (d->queue->serial == d->pkt_serial) { 
            do {
                // 检查中断请求
                if (d->queue->abort_request)
                    return -1;

                // 解码视频和音频数据，调整PTS 
                switch (d->avctx->codec_type) {
                    case AVMEDIA_TYPE_VIDEO:
                        ret = avcodec_receive_frame(d->avctx, frame);
                        if (ret >= 0) { // 成功获取帧（ret >= 0），则调整帧的显示时间戳（pts）。
                            if (decoder_reorder_pts == -1) { // -1 是 auto，则采用估计时间
                                frame->pts = frame->best_effort_timestamp; // 估计的最佳播放时间戳
                            } else if (!decoder_reorder_pts) { // decoder_reorder_pts == 0,  不允许解码器重排，解码时间戳就是显示时间戳
                                frame->pts = frame->pkt_dts;
                            } 
                        } // 确保视频帧具有正确的时间戳，以应对解码器可能对帧进行重新排序的情况。
                        break;
                    case AVMEDIA_TYPE_AUDIO:
                        ret = avcodec_receive_frame(d->avctx, frame);
                        if (ret >= 0) { // 如果成功获取帧，则根据帧的采样率设置时间基（tb）。
                            AVRational tb = (AVRational){ 1, frame->sample_rate };
                            // 如果 pts 有效（不为 AV_NOPTS_VALUE）
                            if (frame->pts != AV_NOPTS_VALUE)
                                frame->pts = av_rescale_q(frame->pts, d->avctx->pkt_timebase, tb); // 从数据包时间基（d->avctx->pkt_timebase）转换为帧的时间基（tb）。
                            else if (d->next_pts != AV_NOPTS_VALUE) // 如果 frame->pts 无效，则使用 d->next_pts 作为基准，并同样进行时间基转换。
                                frame->pts = av_rescale_q(d->next_pts, d->next_pts_tb, tb);
                            // 更新 d->next_pts 和 d->next_pts_tb，为下一帧生成连续的时间戳。
                            if (frame->pts != AV_NOPTS_VALUE) {
                                d->next_pts = frame->pts + frame->nb_samples;
                                d->next_pts_tb = tb;
                            }
                        }
                        break;
                }

                //* 如果解码器已经结束（返回 AVERROR_EOF），则标记解码器为已完成，并刷新解码器缓冲区。
                if (ret == AVERROR_EOF) { // seek操作第一次循环在处理数据包时被置为AVERROR_EOF，最终在下一次循环在这里刷新缓冲区，解码结束退出
                    d->finished = d->pkt_serial;
                    avcodec_flush_buffers(d->avctx);
                    return 0;
                }
                //* 如果成功获取一帧，返回 1。
                if (ret >= 0)
                    return 1;
            } while (ret != AVERROR(EAGAIN));
            //* 成功解码时，直接返回了。 没有直接返回，检查条件时产生EAGAIN，表示解码器中剩余的数据不足以解码出一帧，则尝试获取数据并进行处理
        }

        //* 进行到这里，说明解码器产生了EAGAIN错误，数据不足以解码，尝试通知read线程读取数据，检查是否有保留的数据，没有则尝试重新获取数据， 最后检查seek操作
        // 无论是否因为seek，有无进行上述尝试解码的操作而到这里，都默认数据不足
        //* 并检查是否seek操作，发生seek 操作时，不直接尝试解码， 在这里统一将seek前的packet释放掉(av_packet_unref)
        do {
            if (d->queue->nb_packets == 0) // 如果队列为空，发送信号通知read线程从媒体源读取数据。
                SDL_CondSignal(d->empty_queue_cond);
            if (d->packet_pending) { // packet_pending 有保留的数据包则修改为0表示处理了，后续如果是seek前的数据则直接av_packet_unref，否则进入后续数据处理
                d->packet_pending = 0;
            } else {
                int old_serial = d->pkt_serial;
                // 读取失败直接返回错误，不进行后续操作
                if (packet_queue_get(d->queue, d->pkt, 1, &d->pkt_serial) < 0)
                    return -1;

                // 如果获取数据前后数据包序列号发生变化，说明发生了 seek 操作，需要刷新解码器缓冲区并重置状态。
                // 这里的seek是指 packet_queue_get前后解码器的序列号发生变化，不是尝试解码时检查到的队列序列号和解码器序列号不一致
                if (old_serial != d->pkt_serial) {
                    avcodec_flush_buffers(d->avctx);
                    d->finished = 0;
                    d->next_pts = d->start_pts;
                    d->next_pts_tb = d->start_pts_tb;
                }
            }
            //* 无论是尝试解码时检查到的seek，还是获取数据包时检查到的seek，所需处理都是 av_packet_unref(d->pkt)，释放掉解码器中的packet数据
            // 如果没有发生seek 则暂时不释放解码器的packet数据，直接break
            if (d->queue->serial == d->pkt_serial)
                break;
            av_packet_unref(d->pkt); // 释放数据包的引用。 后续处理数据包时，因为seek而被释放的包会在进行解码检查时发现数据为空报错 
        } while (1);

        //* 处理读取到的数据包
        // 前面读取失败直接返回了，这里肯定读到了数据包
        if (d->avctx->codec_type == AVMEDIA_TYPE_SUBTITLE) { // 字幕帧直接解码
            int got_frame = 0;
            ret = avcodec_decode_subtitle2(d->avctx, sub, &got_frame, d->pkt); // 解码字幕帧。
            if (ret < 0) { 
                ret = AVERROR(EAGAIN); // 数据不足
            } else {
                // 成功解码但数据包为空（got_frame && !d->pkt->data）
                if (got_frame && !d->pkt->data) {
                    d->packet_pending = 1; // 表示数据包需要保留以便后续处理。
                }
                // 成功解码 0 ; 仍有数据但未解码出帧， AVERROR(EAGAIN); 数据包为空，返回 AVERROR_EOF
                ret = got_frame ? 0 : (d->pkt->data ? AVERROR(EAGAIN) : AVERROR_EOF);
            }
            av_packet_unref(d->pkt); // 释放数据包的引用。
        } else { // 视频和音频帧需要将数据送入ffmpeg解码器，下一轮循环尝试解码获取数据
            // 如果数据包有缓冲区（d->pkt->buf）且没有附加数据（!d->pkt->opaque_ref）
            if (d->pkt->buf && !d->pkt->opaque_ref) {
                FrameData *fd;

                // 则为其分配附加数据（FrameData）。
                d->pkt->opaque_ref = av_buffer_allocz(sizeof(*fd));
                if (!d->pkt->opaque_ref)
                    return AVERROR(ENOMEM);
                // 附加数据用于存储数据包的位置信息（pkt_pos）。
                fd = (FrameData*)d->pkt->opaque_ref->data;
                fd->pkt_pos = d->pkt->pos;
            }

            // 将数据包发送到解码器。
            // seek导致前面av_packet_unref之后，d->pkt 变为空数据包。执行send会向解码器发送一个刷新信号，触发解码器输出内部缓冲的帧。
            // 如果解码器内部没有缓冲的帧，avcodec_receive_frame 会返回 AVERROR_EOF，表示解码器已经结束。
            if (avcodec_send_packet(d->avctx, d->pkt) == AVERROR(EAGAIN)) { // 解码器的输入缓冲区已满
                // 如果 avcodec_send_packet 和 avcodec_receive_frame 同时返回 AVERROR(EAGAIN)，记录错误日志，表示 API 使用违规。
                av_log(d->avctx, AV_LOG_ERROR, "Receive_frame and send_packet both returned EAGAIN, which is an API violation.\n");
                d->packet_pending = 1; // 设置 d->packet_pending = 1，保留数据包以便下次发送。
            } else { // 送入packet成功
                av_packet_unref(d->pkt); // 释放数据包的引用。
            }
        }
    }
}

static void decoder_destroy(Decoder *d) {
    av_packet_free(&d->pkt);
    avcodec_free_context(&d->avctx);
}

static void frame_queue_unref_item(Frame *vp)
{
    av_frame_unref(vp->frame);
    avsubtitle_free(&vp->sub);
}

static int frame_queue_init(FrameQueue *f, PacketQueue *pktq, int max_size, int keep_last)
{
    int i;
    memset(f, 0, sizeof(FrameQueue));
    if (!(f->mutex = SDL_CreateMutex())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    if (!(f->cond = SDL_CreateCond())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    f->pktq = pktq;
    f->max_size = FFMIN(max_size, FRAME_QUEUE_SIZE);
    f->keep_last = !!keep_last;
    for (i = 0; i < f->max_size; i++)
        if (!(f->queue[i].frame = av_frame_alloc()))
            return AVERROR(ENOMEM);
    return 0;
}

static void frame_queue_destroy(FrameQueue *f)
{
    int i;
    for (i = 0; i < f->max_size; i++) {
        Frame *vp = &f->queue[i];
        frame_queue_unref_item(vp);
        av_frame_free(&vp->frame);
    }
    SDL_DestroyMutex(f->mutex);
    SDL_DestroyCond(f->cond);
}

static void frame_queue_signal(FrameQueue *f)
{
    SDL_LockMutex(f->mutex);
    SDL_CondSignal(f->cond);
    SDL_UnlockMutex(f->mutex);
}

static Frame *frame_queue_peek(FrameQueue *f)
{
    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

static Frame *frame_queue_peek_next(FrameQueue *f)
{
    return &f->queue[(f->rindex + f->rindex_shown + 1) % f->max_size];
}

static Frame *frame_queue_peek_last(FrameQueue *f)
{
    return &f->queue[f->rindex];
}

//*从队列中获取一个可写的帧（Frame）。用于存储新的帧数据。
// 成功获取到可写帧，返回指向该帧的指针（Frame*）。如果队列被中止（abort_request 为 1），返回 NULL。
static Frame* frame_queue_peek_writable(FrameQueue* f)
{
    /* wait until we have space to put a new frame */
    SDL_LockMutex(f->mutex);
    //* 是否需要等待空间腾出
    while (f->size >= f->max_size &&
           !f->pktq->abort_request) { // 如果队列已满（f->size >= f->max_size）且队列未被中止（!f->pktq->abort_request），
        SDL_CondWait(f->cond, f->mutex); // 等待条件变量 f->cond，直到队列中有空闲空间。
    }
    SDL_UnlockMutex(f->mutex);

    //* 无论是等待结束，还是正常获取之前，都检查队列是否中止
    if (f->pktq->abort_request) 
        return NULL;

    return &f->queue[f->windex];
}

static Frame *frame_queue_peek_readable(FrameQueue *f)
{
    /* wait until we have a readable a new frame */
    SDL_LockMutex(f->mutex);
    while (f->size - f->rindex_shown <= 0 &&
           !f->pktq->abort_request) {
        SDL_CondWait(f->cond, f->mutex);
    }
    SDL_UnlockMutex(f->mutex);

    if (f->pktq->abort_request)
        return NULL;

    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

//* 将一个新帧加入帧队列，并更新队列状态。 通常在调用 frame_queue_peek_writable 获取可写帧并填充数据后调用。
static void frame_queue_push(FrameQueue* f)
{
    if (++f->windex == f->max_size) // 将写索引 f->windex 加 1。
        f->windex = 0; // 如果写索引达到队列的最大容量（f->max_size），则将写索引 f->windex 重置为 0，实现循环队列的效果。
    SDL_LockMutex(f->mutex);
    f->size++;      // 将队列大小 f->size 加 1，表示新帧已加入队列。
    SDL_CondSignal(f->cond);    //  通知可能正在等待的消费者线程
    SDL_UnlockMutex(f->mutex);
}

//* 帧队列的读索引移动到下一帧，并释放当前帧的资源。
static void frame_queue_next(FrameQueue* f)
{
    //* 如果启用了 keep_last 标志且当前帧未显示过，则标记当前帧为已显示并返回。
    if (f->keep_last && !f->rindex_shown) {
        f->rindex_shown = 1;
        return;
    }
    //*  释放当前帧的资源
    frame_queue_unref_item(&f->queue[f->rindex]);
    //* 移动读索引到下一帧
    if (++f->rindex == f->max_size)
        f->rindex = 0;
    //* 更新队列大小并通知生产者线程。
    SDL_LockMutex(f->mutex);
    f->size--;
    SDL_CondSignal(f->cond);
    SDL_UnlockMutex(f->mutex);
}

/* return the number of undisplayed frames in the queue */
//* 返回未播放的帧数
static int frame_queue_nb_remaining(FrameQueue *f)
{
    return f->size - f->rindex_shown;
}

/* return last shown position */
static int64_t frame_queue_last_pos(FrameQueue *f)
{
    Frame *fp = &f->queue[f->rindex];
    if (f->rindex_shown && fp->serial == f->pktq->serial)
        return fp->pos;
    else
        return -1;
}

static void decoder_abort(Decoder *d, FrameQueue *fq)
{
    packet_queue_abort(d->queue);
    frame_queue_signal(fq);
    SDL_WaitThread(d->decoder_tid, NULL);
    d->decoder_tid = NULL;
    packet_queue_flush(d->queue);
}

static inline void fill_rectangle(int x, int y, int w, int h)
{
    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = w;
    rect.h = h;
    if (w && h)
        SDL_RenderFillRect(renderer, &rect);
}

//* 重新分配纹理内存
// 检查当前纹理是否符合要求，如果不符合，则销毁旧纹理并创建一个新的纹理。
// SDL_Texture **texture: 指向纹理指针的指针，用于存储或更新纹理对象。
// Uint32 new_format: 新纹理的像素格式（如 SDL_PIXELFORMAT_ARGB8888）。
// int new_width: 新纹理的宽度。
// int new_height: 新纹理的高度。
// SDL_BlendMode blendmode: 新纹理的混合模式（如 SDL_BLENDMODE_BLEND）。
// int init_texture: 是否初始化纹理（填充为 0）。
static int realloc_texture(SDL_Texture** texture, Uint32 new_format, int new_width, int new_height, SDL_BlendMode blendmode, int init_texture)
{
    Uint32 format;
    int access, w, h;
    //* 检查是否需要重新分配纹理
    if (!*texture  // 纹理指针为空（!*texture)   
        || SDL_QueryTexture(*texture, &format, &access, &w, &h) < 0 //或者查询纹理信息失败（SDL_QueryTexture 返回负值）   
        || new_width != w || new_height != h || new_format != format) { //或者新纹理的宽度、高度或格式与当前纹理不匹配
        //* 重新分配纹理。
        void* pixels;
        int pitch;
        // 释放旧纹理的内存，避免内存泄漏。
        if (*texture)
            SDL_DestroyTexture(*texture); // 如果当前纹理存在，则调用 SDL_DestroyTexture 销毁旧纹理。
        // 创建一个符合要求的新纹理。
        if (!(*texture = SDL_CreateTexture(renderer, new_format, SDL_TEXTUREACCESS_STREAMING, new_width, new_height))) //  创建一个新的纹理，使用指定的格式、宽度和高度。
            return -1;
        // 设置纹理的混合模式
        if (SDL_SetTextureBlendMode(*texture, blendmode) < 0)
            return -1;
        // 初始化纹理
        if (init_texture) {
            if (SDL_LockTexture(*texture, NULL, &pixels, &pitch) < 0) // 锁定纹理，获取像素数据指针 pixels 和行间距 pitch。
                return -1;
            memset(pixels, 0, pitch * new_height); // t 将纹理的像素数据填充为 0（透明黑色）。
            SDL_UnlockTexture(*texture); // 解锁纹理。
        }
        av_log(NULL, AV_LOG_VERBOSE, "Created %dx%d texture with %s.\n", new_width, new_height, SDL_GetPixelFormatName(new_format));
    }
    return 0;
}

//* 根据视频的宽高比（包括像素宽高比）和屏幕的尺寸，计算视频在屏幕上显示的位置和大小。
// scr_xleft, scr_ytop：屏幕左上角的坐标。
// scr_width, scr_height：屏幕的宽度和高度。
// pic_width, pic_height：视频帧的宽度和高度。
// pic_sar：视频的像素宽高比（Sample Aspect Ratio, SAR）
static void calculate_display_rect(SDL_Rect* rect,
                                   int scr_xleft, int scr_ytop, int scr_width, int scr_height,
                                   int pic_width, int pic_height, AVRational pic_sar)
{
    AVRational aspect_ratio = pic_sar;
    int64_t width, height, x, y;

    // 检查像素宽高比
    if (av_cmp_q(aspect_ratio, av_make_q(0, 1)) <= 0) // av_make_q(0, 1) 表示 0:1 的宽高比。
        aspect_ratio = av_make_q(1, 1); // 如果像素宽高比无效（小于或等于 0），则将其设置为 1:1（正方形像素）。
    // 计算实际宽高比
    aspect_ratio = av_mul_q(aspect_ratio, av_make_q(pic_width, pic_height)); // 实际宽高比 = 像素宽高比 × (帧宽度 / 帧高度)

    /* XXX: we suppose the screen has a 1.0 pixel ratio */ // 假设屏幕像素宽高比为 1.0
    height = scr_height;
    width = av_rescale(height, aspect_ratio.num, aspect_ratio.den) & ~1; // 根据实际宽高比计算宽度。& ~1 确保宽度为偶数（避免某些编解码器或渲染器的问题）。
    // 调整宽度和高度
    if (width > scr_width) { // 如果计算出的宽度超过屏幕宽度，则调整宽度和高度。
        width = scr_width;  // 将宽度设置为屏幕宽度（width = scr_width）
        height = av_rescale(width, aspect_ratio.den, aspect_ratio.num) & ~1; // 根据实际宽高比重新计算高度，并确保高度为偶数。
    }
    // 计算显示位置
    x = (scr_width - width) / 2;
    y = (scr_height - height) / 2;
    // 设置显示区域
    rect->x = scr_xleft + x;        // 屏幕左上角的坐标。
    rect->y = scr_ytop  + y;
    rect->w = FFMAX((int)width,  1); // 屏幕的宽度和高度。确保宽度和高度至少为 1，避免无效值。
    rect->h = FFMAX((int)height, 1); 
}

static void get_sdl_pix_fmt_and_blendmode(int format, Uint32 *sdl_pix_fmt, SDL_BlendMode *sdl_blendmode)
{
    int i;
    *sdl_blendmode = SDL_BLENDMODE_NONE;
    *sdl_pix_fmt = SDL_PIXELFORMAT_UNKNOWN;
    if (format == AV_PIX_FMT_RGB32   ||
        format == AV_PIX_FMT_RGB32_1 ||
        format == AV_PIX_FMT_BGR32   ||
        format == AV_PIX_FMT_BGR32_1)
        *sdl_blendmode = SDL_BLENDMODE_BLEND;
    for (i = 0; i < FF_ARRAY_ELEMS(sdl_texture_format_map) - 1; i++) {
        if (format == sdl_texture_format_map[i].format) {
            *sdl_pix_fmt = sdl_texture_format_map[i].texture_fmt;
            return;
        }
    }
}

//* 将 AVFrame 数据上传到 SDL_Texture 
// 根据帧的像素格式和行间距，将帧数据上传到纹理中。
// SDL_Texture **tex: 指向纹理指针的指针，用于存储或更新纹理对象。
// AVFrame *frame: 指向 AVFrame 结构体的指针，包含需要上传的帧数据。
static int upload_texture(SDL_Texture** tex, AVFrame* frame)
{
    int ret = 0;
    // sdl_pix_fmt 和 sdl_blendmode，用于存储 SDL 像素格式和混合模式。
    Uint32 sdl_pix_fmt;
    SDL_BlendMode sdl_blendmode;
    // 获取 SDL 像素格式和混合模式, 确保纹理的像素格式和混合模式与帧数据匹配。
    get_sdl_pix_fmt_and_blendmode(frame->format, &sdl_pix_fmt, &sdl_blendmode);
    //  重新分配纹理内存
    if (realloc_texture(tex, sdl_pix_fmt == SDL_PIXELFORMAT_UNKNOWN ? SDL_PIXELFORMAT_ARGB8888 : sdl_pix_fmt, frame->width, frame->height, sdl_blendmode, 0) < 0)
        return -1;
    // 根据帧的像素格式和行间距，将帧数据上传到纹理中
    switch (sdl_pix_fmt) {
    case SDL_PIXELFORMAT_IYUV: // YUV 格式（SDL_PIXELFORMAT_IYUV）
        // 如果行间距（linesize）为正，则直接调用 SDL_UpdateYUVTexture 上传 YUV 数据。
        if (frame->linesize[0] > 0 && frame->linesize[1] > 0 && frame->linesize[2] > 0) {
            ret = SDL_UpdateYUVTexture(*tex, NULL, frame->data[0], frame->linesize[0],
                frame->data[1], frame->linesize[1],
                frame->data[2], frame->linesize[2]);
            // 如果行间距为负，则调整数据指针和行间距，以支持从下到上的图像数据。
        } else if (frame->linesize[0] < 0 && frame->linesize[1] < 0 && frame->linesize[2] < 0) {
            ret = SDL_UpdateYUVTexture(*tex, NULL, frame->data[0] + frame->linesize[0] * (frame->height - 1), -frame->linesize[0],
                frame->data[1] + frame->linesize[1] * (AV_CEIL_RSHIFT(frame->height, 1) - 1), -frame->linesize[1],
                frame->data[2] + frame->linesize[2] * (AV_CEIL_RSHIFT(frame->height, 1) - 1), -frame->linesize[2]);
        } else { // 如果行间距混合了正负值，则记录错误并返回 -1。
            av_log(NULL, AV_LOG_ERROR, "Mixed negative and positive linesizes are not supported.\n");
            return -1;
        }
        break;
    default: // 其他格式:
        if (frame->linesize[0] < 0) { // 如果行间距为负，则调整数据指针和行间距，以支持从下到上的图像数据。
            ret = SDL_UpdateTexture(*tex, NULL, frame->data[0] + frame->linesize[0] * (frame->height - 1), -frame->linesize[0]);
        } else { // 否则，直接调用 SDL_UpdateTexture 上传数据。
            ret = SDL_UpdateTexture(*tex, NULL, frame->data[0], frame->linesize[0]);
        }
        break;
    }
    return ret;
}

static enum AVColorSpace sdl_supported_color_spaces[] = {
    AVCOL_SPC_BT709,
    AVCOL_SPC_BT470BG,
    AVCOL_SPC_SMPTE170M,
    AVCOL_SPC_UNSPECIFIED,
};

static void set_sdl_yuv_conversion_mode(AVFrame *frame)
{
#if SDL_VERSION_ATLEAST(2,0,8)
    SDL_YUV_CONVERSION_MODE mode = SDL_YUV_CONVERSION_AUTOMATIC;
    if (frame && (frame->format == AV_PIX_FMT_YUV420P || frame->format == AV_PIX_FMT_YUYV422 || frame->format == AV_PIX_FMT_UYVY422)) {
        if (frame->color_range == AVCOL_RANGE_JPEG)
            mode = SDL_YUV_CONVERSION_JPEG;
        else if (frame->colorspace == AVCOL_SPC_BT709)
            mode = SDL_YUV_CONVERSION_BT709;
        else if (frame->colorspace == AVCOL_SPC_BT470BG || frame->colorspace == AVCOL_SPC_SMPTE170M)
            mode = SDL_YUV_CONVERSION_BT601;
    }
    SDL_SetYUVConversionMode(mode); /* FIXME: no support for linear transfer */
#endif
}

//* 渲染和显示视频帧
// 将解码后的视频帧和字幕渲染到屏幕上
static void video_image_display(VideoState* is)
{
    Frame *vp;
    Frame *sp = NULL;
    SDL_Rect rect;

    //* 从视频帧队列 is->pictq 中获取当前帧vp。
    vp = frame_queue_peek_last(&is->pictq);
    // Vulkan 渲染器支持
    if (vk_renderer) { // Vulkan 渲染器（vk_renderer）已启用，则调用 vk_renderer_display 渲染视频帧，并直接返回。
        vk_renderer_display(vk_renderer, vp->frame);
        return;
    }

    //* 处理字幕
    if (is->subtitle_st) { //如果存在字幕流（is->subtitle_st），并且字幕队列 is->subpq 中有字幕帧：
        if (frame_queue_nb_remaining(&is->subpq) > 0) {
            sp = frame_queue_peek(&is->subpq); // 获取当前字幕帧 sp。

            //* 检查当前视频帧的时间戳 vp->pts 是否大于等于字幕帧的开始显示时间。
            if (vp->pts >= sp->pts + ((float)sp->sub.start_display_time / 1000)) {
                //* 如果字幕帧尚未上传到纹理（sp->uploaded == 0）
                if (!sp->uploaded) {
                    // 存储纹理的像素数据和行间距。
                    uint8_t* pixels[4];
                    int pitch[4];
                    int i;
                    // 存储纹理的像素数据和行间距。
                    if (!sp->width || !sp->height) { 
                        sp->width = vp->width;
                        sp->height = vp->height;
                    }
                    // 重新分配字幕纹理的内存。 如果分配失败，则直接返回。
                    if (realloc_texture(&is->sub_texture, SDL_PIXELFORMAT_ARGB8888, sp->width, sp->height, SDL_BLENDMODE_BLEND, 1) < 0)
                        return; // 使用 SDL_PIXELFORMAT_ARGB8888 格式（32 位带透明度的像素格式）。

                    //* 处理字幕矩形
                    for (i = 0; i < sp->sub.num_rects; i++) {
                        //* 确保字幕矩形的坐标和尺寸合法。
                        // 遍历字幕帧中的所有矩形（sp->sub.num_rects）。
                        AVSubtitleRect* sub_rect = sp->sub.rects[i];

                        // 使用 av_clip 函数将每个矩形的坐标和尺寸限制在字幕纹理的范围内，避免越界。
                        sub_rect->x = av_clip(sub_rect->x, 0, sp->width);
                        sub_rect->y = av_clip(sub_rect->y, 0, sp->height);
                        sub_rect->w = av_clip(sub_rect->w, 0, sp->width  - sub_rect->x);
                        sub_rect->h = av_clip(sub_rect->h, 0, sp->height - sub_rect->y);

                        //* 初始化转换上下文
                        // 获取或初始化一个图像转换上下文（is->sub_convert_ctx）。
                        is->sub_convert_ctx = sws_getCachedContext(is->sub_convert_ctx,
                            sub_rect->w, sub_rect->h, AV_PIX_FMT_PAL8, // 从 AV_PIX_FMT_PAL8（8 位调色板格式）
                            sub_rect->w, sub_rect->h, AV_PIX_FMT_BGRA, // 转换为 AV_PIX_FMT_BGRA（32 位带透明度的像素格式）。
                            0, NULL, NULL, NULL);
                        if (!is->sub_convert_ctx) { // 如果转换上下文初始化失败，则记录错误并返回。
                            av_log(NULL, AV_LOG_FATAL, "Cannot initialize the conversion context\n");
                            return;
                        }
                        //* 上传字幕矩形到纹理
                        if (!SDL_LockTexture(is->sub_texture, (SDL_Rect*)sub_rect, (void**)pixels, pitch)) { // 锁定字幕纹理，获取像素数据指针 pixels 和行间距 pitch。
                            // 将字幕矩形的像素数据从 AV_PIX_FMT_PAL8 转换为 AV_PIX_FMT_BGRA，并写入纹理。
                            sws_scale(is->sub_convert_ctx, (const uint8_t* const*)sub_rect->data, sub_rect->linesize,
                                0, sub_rect->h, pixels, pitch);
                            // 使用 SDL_UnlockTexture 解锁纹理。
                            SDL_UnlockTexture(is->sub_texture);
                        }
                    }
                    //* 标记字幕帧为已上传
                    sp->uploaded = 1;
                }
            } else
                //* 如果当前视频帧的时间戳小于字幕帧的开始显示时间，则将 sp 置为 NULL。
                sp = NULL;
        }
    }

    //* 计算视频帧的显示区域   考虑窗口尺寸、视频帧尺寸和像素宽高比（vp->sar）。    确保视频帧在窗口中正确显示，避免拉伸或变形。
    calculate_display_rect(&rect, is->xleft, is->ytop, is->width, is->height, vp->width, vp->height, vp->sar);
    //* 设置 YUV 转换模式    设置 YUV 到 RGB 的转换模式，根据视频帧的格式进行调整。    确保视频帧的颜色空间正确转换。
    set_sdl_yuv_conversion_mode(vp->frame);

    //* 上传视频帧到纹理
    // 将视频帧数据上传到 GPU 纹理，以便后续渲染。
    if (!vp->uploaded) {
        if (upload_texture(&is->vid_texture, vp->frame) < 0) { // 如果视频帧尚未上传到纹理（vp->uploaded == 0），则调用 upload_texture 将视频帧上传到纹理。
            // 上传失败，则恢复 YUV 转换模式并返回。
            set_sdl_yuv_conversion_mode(NULL);
            return;
        }
        // 标记视频帧为已上传（vp->uploaded = 1），并设置垂直翻转标志（vp->flip_v）。
        vp->uploaded = 1;
        vp->flip_v = vp->frame->linesize[0] < 0;
    }

    //* 渲染视频帧  
    SDL_RenderCopyEx(renderer, is->vid_texture, NULL, &rect, 0, NULL, vp->flip_v ? SDL_FLIP_VERTICAL : 0); // 将视频纹理渲染到屏幕上，考虑垂直翻转标志。
    set_sdl_yuv_conversion_mode(NULL); // 恢复 YUV 转换模式。

    //* 渲染字幕
    // 将字幕纹理叠加到视频帧上，确保字幕在正确的位置和比例显示。
    if (sp) {
#if USE_ONEPASS_SUBTITLE_RENDER 
        // 单次渲染模式
        // 简化字幕渲染逻辑，适用于字幕纹理与视频帧显示区域完全匹配的情况。
        SDL_RenderCopy(renderer, is->sub_texture, NULL, &rect); // 将字幕纹理复制到渲染器的目标区域。
#else // 逐矩形渲染模式
        // 逐个渲染字幕矩形，考虑缩放比例（xratio 和 yratio）。
        int i;
        // 计算缩放比例：
        double xratio = (double)rect.w / (double)sp->width;
        double yratio = (double)rect.h / (double)sp->height;
        // 遍历字幕帧中的所有矩形（sp->sub.num_rects）：
        for (i = 0; i < sp->sub.num_rects; i++) {
            SDL_Rect *sub_rect = (SDL_Rect*)sp->sub.rects[i];
            SDL_Rect target = {.x = rect.x + sub_rect->x * xratio,  // target.x 和 target.y：字幕矩形在视频帧显示区域中的位置。
                               .y = rect.y + sub_rect->y * yratio,
                               .w = sub_rect->w * xratio,   // target.w 和 target.h：字幕矩形在视频帧显示区域中的尺寸。
                               .h = sub_rect->h * yratio};
            SDL_RenderCopy(renderer, is->sub_texture, sub_rect, &target); //  将字幕矩形渲染到目标区域。
        }   
#endif
    }
}

static inline int compute_mod(int a, int b)
{
    return a < 0 ? a%b + b : a%b;
}

static void video_audio_display(VideoState *s)
{
    int i, i_start, x, y1, y, ys, delay, n, nb_display_channels;
    int ch, channels, h, h2;
    int64_t time_diff;
    int rdft_bits, nb_freq;

    for (rdft_bits = 1; (1 << rdft_bits) < 2 * s->height; rdft_bits++)
        ;
    nb_freq = 1 << (rdft_bits - 1);

    /* compute display index : center on currently output samples */
    channels = s->audio_tgt.ch_layout.nb_channels;
    nb_display_channels = channels;
    if (!s->paused) {
        int data_used= s->show_mode == SHOW_MODE_WAVES ? s->width : (2*nb_freq);
        n = 2 * channels;
        delay = s->audio_write_buf_size;
        delay /= n;

        /* to be more precise, we take into account the time spent since
           the last buffer computation */ //* 为了更加精确，考虑了自上次缓冲区计算以来花费的时间
        if (audio_callback_time) {
            time_diff = av_gettime_relative() - audio_callback_time;
            delay -= (time_diff * s->audio_tgt.freq) / 1000000;
        }

        delay += 2 * data_used;
        if (delay < data_used)
            delay = data_used;

        i_start= x = compute_mod(s->sample_array_index - delay * channels, SAMPLE_ARRAY_SIZE);
        if (s->show_mode == SHOW_MODE_WAVES) {
            h = INT_MIN;
            for (i = 0; i < 1000; i += channels) {
                int idx = (SAMPLE_ARRAY_SIZE + x - i) % SAMPLE_ARRAY_SIZE;
                int a = s->sample_array[idx];
                int b = s->sample_array[(idx + 4 * channels) % SAMPLE_ARRAY_SIZE];
                int c = s->sample_array[(idx + 5 * channels) % SAMPLE_ARRAY_SIZE];
                int d = s->sample_array[(idx + 9 * channels) % SAMPLE_ARRAY_SIZE];
                int score = a - d;
                if (h < score && (b ^ c) < 0) {
                    h = score;
                    i_start = idx;
                }
            }
        }

        s->last_i_start = i_start;
    } else {
        i_start = s->last_i_start;
    }

    if (s->show_mode == SHOW_MODE_WAVES) {
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        /* total height for one channel */
        h = s->height / nb_display_channels;
        /* graph height / 2 */
        h2 = (h * 9) / 20;
        for (ch = 0; ch < nb_display_channels; ch++) {
            i = i_start + ch;
            y1 = s->ytop + ch * h + (h / 2); /* position of center line */
            for (x = 0; x < s->width; x++) {
                y = (s->sample_array[i] * h2) >> 15;
                if (y < 0) {
                    y = -y;
                    ys = y1 - y;
                } else {
                    ys = y1;
                }
                fill_rectangle(s->xleft + x, ys, 1, y);
                i += channels;
                if (i >= SAMPLE_ARRAY_SIZE)
                    i -= SAMPLE_ARRAY_SIZE;
            }
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);

        for (ch = 1; ch < nb_display_channels; ch++) {
            y = s->ytop + ch * h;
            fill_rectangle(s->xleft, y, s->width, 1);
        }
    } else {
        int err = 0;
        if (realloc_texture(&s->vis_texture, SDL_PIXELFORMAT_ARGB8888, s->width, s->height, SDL_BLENDMODE_NONE, 1) < 0)
            return;

        if (s->xpos >= s->width)
            s->xpos = 0;
        nb_display_channels= FFMIN(nb_display_channels, 2);
        if (rdft_bits != s->rdft_bits) {
            const float rdft_scale = 1.0;
            av_tx_uninit(&s->rdft);
            av_freep(&s->real_data);
            av_freep(&s->rdft_data);
            s->rdft_bits = rdft_bits;
            s->real_data = av_malloc_array(nb_freq, 4 *sizeof(*s->real_data));
            s->rdft_data = av_malloc_array(nb_freq + 1, 2 *sizeof(*s->rdft_data));
            err = av_tx_init(&s->rdft, &s->rdft_fn, AV_TX_FLOAT_RDFT,
                             0, 1 << rdft_bits, &rdft_scale, 0);
        }
        if (err < 0 || !s->rdft_data) {
            av_log(NULL, AV_LOG_ERROR, "Failed to allocate buffers for RDFT, switching to waves display\n");
            s->show_mode = SHOW_MODE_WAVES;
        } else {
            float *data_in[2];
            AVComplexFloat *data[2];
            SDL_Rect rect = {.x = s->xpos, .y = 0, .w = 1, .h = s->height};
            uint32_t *pixels;
            int pitch;
            for (ch = 0; ch < nb_display_channels; ch++) {
                data_in[ch] = s->real_data + 2 * nb_freq * ch;
                data[ch] = s->rdft_data + nb_freq * ch;
                i = i_start + ch;
                for (x = 0; x < 2 * nb_freq; x++) {
                    double w = (x-nb_freq) * (1.0 / nb_freq);
                    data_in[ch][x] = s->sample_array[i] * (1.0 - w * w);
                    i += channels;
                    if (i >= SAMPLE_ARRAY_SIZE)
                        i -= SAMPLE_ARRAY_SIZE;
                }
                s->rdft_fn(s->rdft, data[ch], data_in[ch], sizeof(float));
                data[ch][0].im = data[ch][nb_freq].re;
                data[ch][nb_freq].re = 0;
            }
            /* Least efficient way to do this, we should of course
             * directly access it but it is more than fast enough. */
            if (!SDL_LockTexture(s->vis_texture, &rect, (void **)&pixels, &pitch)) {
                pitch >>= 2;
                pixels += pitch * s->height;
                for (y = 0; y < s->height; y++) {
                    double w = 1 / sqrt(nb_freq);
                    int a = sqrt(w * sqrt(data[0][y].re * data[0][y].re + data[0][y].im * data[0][y].im));
                    int b = (nb_display_channels == 2 ) ? sqrt(w * hypot(data[1][y].re, data[1][y].im))
                                                        : a;
                    a = FFMIN(a, 255);
                    b = FFMIN(b, 255);
                    pixels -= pitch;
                    *pixels = (a << 16) + (b << 8) + ((a+b) >> 1);
                }
                SDL_UnlockTexture(s->vis_texture);
            }
            SDL_RenderCopy(renderer, s->vis_texture, NULL, NULL);
        }
        if (!s->paused)
            s->xpos++;
    }
}

static void stream_component_close(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecParameters *codecpar;

    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return;
    codecpar = ic->streams[stream_index]->codecpar;

    switch (codecpar->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        decoder_abort(&is->auddec, &is->sampq);
        SDL_CloseAudioDevice(audio_dev);
        decoder_destroy(&is->auddec);
        swr_free(&is->swr_ctx);
        av_freep(&is->audio_buf1);
        is->audio_buf1_size = 0;
        is->audio_buf = NULL;

        if (is->rdft) {
            av_tx_uninit(&is->rdft);
            av_freep(&is->real_data);
            av_freep(&is->rdft_data);
            is->rdft = NULL;
            is->rdft_bits = 0;
        }
        break;
    case AVMEDIA_TYPE_VIDEO:
        decoder_abort(&is->viddec, &is->pictq);
        decoder_destroy(&is->viddec);
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        decoder_abort(&is->subdec, &is->subpq);
        decoder_destroy(&is->subdec);
        break;
    default:
        break;
    }

    ic->streams[stream_index]->discard = AVDISCARD_ALL;
    switch (codecpar->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        is->audio_st = NULL;
        is->audio_stream = -1;
        break;
    case AVMEDIA_TYPE_VIDEO:
        is->video_st = NULL;
        is->video_stream = -1;
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        is->subtitle_st = NULL;
        is->subtitle_stream = -1;
        break;
    default:
        break;
    }
}

//* 清理和释放与视频流相关的资源
static void stream_close(VideoState *is)
{
    /* XXX: use a special url_shutdown call to abort parse cleanly */
    //* 中断read线程
    is->abort_request = 1;
    //* 等待read线程退出。确保在继续清理其他资源之前，read线程已经安全退出。
    SDL_WaitThread(is->read_tid, NULL);

    /* close each stream */
    //* 逐个关闭音频、视频和字幕流
    if ( is->audio_stream >= 0 )
        stream_component_close(is, is->audio_stream);
    if (is->video_stream >= 0)
        stream_component_close(is, is->video_stream);
    if (is->subtitle_stream >= 0)
        stream_component_close(is, is->subtitle_stream);

    //* 关闭输入格式上下文 is->ic
    avformat_close_input(&is->ic);

    //* 销毁视频、音频和字幕的包队列
    packet_queue_destroy(&is->videoq);
    packet_queue_destroy(&is->audioq);
    packet_queue_destroy(&is->subtitleq);

    /* free all pictures */
    //* 来释放图像、音频样本和字幕的帧队列
    frame_queue_destroy(&is->pictq);
    frame_queue_destroy(&is->sampq);
    frame_queue_destroy(&is->subpq);

    //* 销毁用于read线程控制的条件变量，确保不再有挂起的线程等待这个条件。
    SDL_DestroyCond(is->continue_read_thread);

    //* 释放转换上下文
    sws_freeContext(is->sub_convert_ctx);

    //* 释放输入文件
    av_free(is->filename);

    //* 销毁视频、可视化和字幕相关的纹理图形资源
    if ( is->vis_texture )
        SDL_DestroyTexture(is->vis_texture);
    if (is->vid_texture)
        SDL_DestroyTexture(is->vid_texture);
    if (is->sub_texture)
        SDL_DestroyTexture(is->sub_texture);

    //* 释放 VideoState 输入流结构体本身，完成资源清理
    av_free(is);
}

static void do_exit(VideoState *is)
{
    if (is) {
        stream_close(is);
    }
    if (renderer)
        SDL_DestroyRenderer(renderer);
    if (vk_renderer)
        vk_renderer_destroy(vk_renderer);
    if (window)
        SDL_DestroyWindow(window);
    uninit_opts();
    for (int i = 0; i < nb_vfilters; i++)
        av_freep(&vfilters_list[i]);
    av_freep(&vfilters_list);
    av_freep(&video_codec_name);
    av_freep(&audio_codec_name);
    av_freep(&subtitle_codec_name);
    av_freep(&input_filename);
    avformat_network_deinit();
    if (show_status)
        printf("\n");
    SDL_Quit();
    av_log(NULL, AV_LOG_QUIET, "%s", "");
    exit(0);
}

static void sigterm_handler(int sig)
{
    exit(123);
}

//* 根据视频流的宽度、高度和宽高比（SAR）计算默认的窗口大小
static void set_default_window_size(int width, int height, AVRational sar)
{
    SDL_Rect rect;
    // 用户如果未指定（值为 0），则将最大窗口尺寸设置为 INT_MAX（即无限制）。
    int max_width = screen_width ? screen_width : INT_MAX;
    int max_height = screen_height ? screen_height : INT_MAX;
    // 如果最大宽度和最大高度均为 INT_MAX，则将最大高度设置为视频流的高度。
    if (max_width == INT_MAX && max_height == INT_MAX)
        max_height = height;
    // 根据视频流的宽度、高度和宽高比计算显示矩形。
    calculate_display_rect(&rect, 0, 0, max_width, max_height, width, height, sar);
    // 存储默认窗口的宽度和高度。
    default_width = rect.w;
    default_height = rect.h;
}

//* 初始化视频窗口
// 设置窗口的标题、尺寸、位置以及全屏模式，并显示窗口。
static int video_open(VideoState* is)
{
    int w,h;

    //* 如果 screen_width 和 screen_height 已定义（非零），则使用它们作为窗口的宽度和高度。
    // 确保窗口尺寸有一个合理的默认值，同时允许用户通过 screen_width 和 screen_height 自定义窗口尺寸。
    w = screen_width ? screen_width : default_width;
    h = screen_height ? screen_height : default_height;

    //* 设置窗口标题
    if (!window_title)
        window_title = input_filename;
    SDL_SetWindowTitle(window, window_title);

    //* 设置窗口尺寸和位置
    SDL_SetWindowSize(window, w, h);
    SDL_SetWindowPosition(window, screen_left, screen_top); // 设置窗口的位置，位置由 screen_left 和 screen_top 决定。
    //* 全屏模式
    if (is_full_screen)
        SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP); // SDL_WINDOW_FULLSCREEN_DESKTOP 模式，确保全屏模式兼容当前桌面分辨率。
    //* 显示窗口。
    SDL_ShowWindow(window);

    //*  更新 VideoState 的尺寸信息
    is->width = w;
    is->height = h;

    return 0;
}

/* display the current picture, if any */
//* 根据播放器的状态（如是否有视频流、音频流以及当前的显示模式）来决定如何渲染画面。
static void video_display(VideoState* is)
{
    //*保在显示画面之前，视频窗口已经正确初始化。
    if (!is->width) //* 如果 is->width 为 0，表示视频窗口尚未初始化。
        video_open(is); //*  初始化视频显示窗口。

    //* 清空渲染器
    // 在渲染新画面之前，清除之前的画面内容，避免画面残留。
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // 渲染器的绘制颜色为黑色（RGBA: 0, 0, 0, 255）。
    SDL_RenderClear(renderer); //  清空渲染器，将整个窗口填充为黑色。
    //* 根据显示模式渲染内容
    if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO)// 音频流存在, 且当前显示模式不是视频模式
        video_audio_display(is);  // 渲染音频波形
    else if (is->video_st) // 视频流
        video_image_display(is); // 渲染视频
    // 将渲染器的内容提交到窗口，显示最终的画面。
    SDL_RenderPresent(renderer);
}

//* 获取时钟的当前时间。
static double get_clock(Clock* c)
{
    if (*c->queue_serial != c->serial)
        return NAN;
    if (c->paused) {
        return c->pts;
    } else {
        double time = av_gettime_relative() / 1000000.0;
        return c->pts_drift + time - (time - c->last_updated) * (1.0 - c->speed);
    }
}

//* 更新时钟的状态到给定的状态
static void set_clock_at(Clock *c, double pts, int serial, double time)
{
    c->pts = pts;
    c->last_updated = time;
    c->pts_drift = c->pts - time;
    c->serial = serial;
}

static void set_clock(Clock *c, double pts, int serial)
{
    double time = av_gettime_relative() / 1000000.0;
    set_clock_at(c, pts, serial, time);
}

static void set_clock_speed(Clock *c, double speed)
{
    set_clock(c, get_clock(c), c->serial);
    c->speed = speed;
}

static void init_clock(Clock *c, int *queue_serial)
{
    c->speed = 1.0;
    c->paused = 0;
    c->queue_serial = queue_serial;
    set_clock(c, NAN, -1);
}

//* 将一个时钟（c）同步到另一个从时钟（slave）
static void sync_clock_to_slave(Clock* c, Clock* slave)
{
    double clock = get_clock(c); // 获取主时钟的当前时间。
    double slave_clock = get_clock(slave); // 获取从时钟的当前时间。
    // 检查是否需要将主时钟同步到从时钟。
    // 从时钟的时间有效（!isnan(slave_clock)）
    // 主时钟的时间无效（isnan(clock)）
    // 或主从时钟的时间差超过阈值（fabs(clock - slave_clock) > AV_NOSYNC_THRESHOLD）。
    if (!isnan(slave_clock) && (isnan(clock) || fabs(clock - slave_clock) > AV_NOSYNC_THRESHOLD))
        set_clock(c, slave_clock, slave->serial);
}

//* 确定当前的主同步类型（视频时钟、音频时钟或外部时钟）。
static int get_master_sync_type(VideoState* is)
{
    if (is->av_sync_type == AV_SYNC_VIDEO_MASTER) {
        //如果以视频时钟为主时钟，检查视频流是否存在，如果不存在，退回音频时钟为主时钟
        if (is->video_st)
            return AV_SYNC_VIDEO_MASTER;
        else
            return AV_SYNC_AUDIO_MASTER;
    } else if (is->av_sync_type == AV_SYNC_AUDIO_MASTER) {
        //如果以音频时钟为主时钟，检查音频流是否存在，如果不存在，退回外部时钟为主时钟
        if (is->audio_st)
            return AV_SYNC_AUDIO_MASTER;
        else
            return AV_SYNC_EXTERNAL_CLOCK;
    } else {
        // 否则直接采用外部时钟为主时钟
        return AV_SYNC_EXTERNAL_CLOCK;
    }
}

/* get the current master clock value */
static double get_master_clock(VideoState *is)
{
    double val;

    switch (get_master_sync_type(is)) {
        case AV_SYNC_VIDEO_MASTER:
            val = get_clock(&is->vidclk);
            break;
        case AV_SYNC_AUDIO_MASTER:
            val = get_clock(&is->audclk);
            break;
        default:
            val = get_clock(&is->extclk);
            break;
    }
    return val;
}

//* 根据音视频队列中的包数量来动态调整外部时钟的速度，以确保音视频同步和流畅播放。
static void check_external_clock_speed(VideoState* is)
{
    //* 当音视频队列中的包数量较少时，说明数据不足，需要降低外部时钟的速度，以避免播放过快导致卡顿或数据不足。
   if (is->video_stream >= 0 && is->videoq.nb_packets <= EXTERNAL_CLOCK_MIN_FRAMES ||  // 视频流存在, 且视频队列中的包数量小于等于 EXTERNAL_CLOCK_MIN_FRAMES
       is->audio_stream >= 0 && is->audioq.nb_packets <= EXTERNAL_CLOCK_MIN_FRAMES) {  // 音频流存在, 且音频队列中的包数量小于等于 EXTERNAL_CLOCK_MIN_FRAMES
        // 降低外部时钟的速度，减少的步长为 EXTERNAL_CLOCK_SPEED_STEP。
        // 使用 FFMAX 确保时钟速度不会低于 EXTERNAL_CLOCK_SPEED_MIN。
       set_clock_speed(&is->extclk, FFMAX(EXTERNAL_CLOCK_SPEED_MIN, is->extclk.speed - EXTERNAL_CLOCK_SPEED_STEP));
   } //* 当音视频队列中的包数量较多时，说明数据充足，可以加快外部时钟的速度，以避免播放过慢导致数据堆积。
   else if ((is->video_stream < 0 || is->videoq.nb_packets > EXTERNAL_CLOCK_MAX_FRAMES) && // 视频流不存在或者视频队列中的包数量大于 EXTERNAL_CLOCK_MAX_FRAMES
       (is->audio_stream < 0 || is->audioq.nb_packets > EXTERNAL_CLOCK_MAX_FRAMES)) { // 音频流不存在或者音频队列中的包数量大于 EXTERNAL_CLOCK_MAX_FRAMES。
        // 提高外部时钟的速度，增加的步长为 EXTERNAL_CLOCK_SPEED_STEP。
        // 使用 FFMIN 确保时钟速度不会超过 EXTERNAL_CLOCK_SPEED_MAX。
       set_clock_speed(&is->extclk, FFMIN(EXTERNAL_CLOCK_SPEED_MAX, is->extclk.speed + EXTERNAL_CLOCK_SPEED_STEP));
   } else { //* 既不满足降低速度的条件，也不满足提高速度的条件。 将外部时钟的速度逐步调整回正常值（1.0）。
        // 当音视频队列中的包数量处于合理范围内时，逐步将外部时钟的速度调整回正常值，以确保播放的稳定性。
       double speed = is->extclk.speed;
       if (speed != 1.0)
           set_clock_speed(&is->extclk, speed + EXTERNAL_CLOCK_SPEED_STEP * (1.0 - speed) / fabs(1.0 - speed));
       // 使用公式 speed + EXTERNAL_CLOCK_SPEED_STEP * (1.0 - speed) / fabs(1.0 - speed) 来平滑调整速度：
            // 如果当前速度 speed 大于 1.0，则逐步减小速度。
            // 如果当前速度 speed 小于 1.0，则逐步增加速度。
   }
}

/* seek in the stream */
//* 触发播放器寻址（Seeking）操作的函数。它设置寻址请求的参数，并通知读取线程执行寻址操作。
// v寻址操作通常用于跳转到媒体文件的指定位置（如用户拖动进度条时）
// VideoState *is：指向播放器全局状态（VideoState）的指针。
// int64_t pos：目标位置（时间戳或字节位置）。
// int64_t rel：相对偏移量（相对于 pos 的偏移）。
// int by_bytes：是否按字节寻址（1 表示按字节寻址，0 表示按时间寻址）。
static void stream_seek(VideoState* is, int64_t pos, int64_t rel, int by_bytes)
{
    //* 是否有未处理的寻址请求
    if (!is->seek_req) { // 为 0，表示当前没有未处理的寻址请求，可以设置新的寻址请求。
        is->seek_pos = pos; // 设置目标位置（pos）。 如果按时间寻址，pos 表示目标时间戳（以微秒为单位）。如果按字节寻址，pos 表示目标字节位置。
        is->seek_rel = rel; // 设置相对偏移量（rel）。 rel 是相对于 pos 的偏移量，用于调整寻址范围。
        is->seek_flags &= ~AVSEEK_FLAG_BYTE;  // 首先清除 AVSEEK_FLAG_BYTE 标志（is->seek_flags &= ~AVSEEK_FLAG_BYTE） ; 
        if (by_bytes) // 如果 by_bytes 为 1，则设置 AVSEEK_FLAG_BYTE 标志（is->seek_flags |= AVSEEK_FLAG_BYTE），表示按字节寻址。
            is->seek_flags |= AVSEEK_FLAG_BYTE;
        is->seek_req = 1; // 将寻址请求标志设置为 1，表示有一个新的寻址请求需要处理。
        SDL_CondSignal(is->continue_read_thread); // 通知读取线程（read_thread）执行寻址操作。
    }
}

/* pause or resume the video */
//* 切换播放器暂停/恢复状态的函数。
// 通过调整播放器的时钟状态和暂停标志来实现暂停和恢复功能。
static void stream_toggle_pause(VideoState* is)
{
    //* 如果播放器当前处于暂停状态（is->paused 为真），则执行恢复逻辑。
    if (is->paused) {
        // 将暂停的时间补偿到 frame_timer 中。
        is->frame_timer += av_gettime_relative() / 1000000.0 - is->vidclk.last_updated; // 在暂停期间，视频时钟（vidclk）没有更新，因此需要将暂停的时间累加到 frame_timer 中。
        if (is->read_pause_return != AVERROR(ENOSYS)) { // 暂停功能可用
            is->vidclk.paused = 0;
        }
        // 更新视频时钟的状态。
        set_clock(&is->vidclk, get_clock(&is->vidclk), is->vidclk.serial);
    }
    //* 无论暂停还是恢复，都需要更新外部时钟（extclk）。
    set_clock(&is->extclk, get_clock(&is->extclk), is->extclk.serial);
    //*  切换暂停状态
    // 将 is->paused 取反，表示切换暂停/恢复状态。
    // 同时更新音频时钟（audclk）、视频时钟（vidclk）和外部时钟（extclk）的暂停标志。
    is->paused = is->audclk.paused = is->vidclk.paused = is->extclk.paused = !is->paused;
}

static void toggle_pause(VideoState *is)
{
    stream_toggle_pause(is);
    is->step = 0;
}

static void toggle_mute(VideoState *is)
{
    is->muted = !is->muted;
}

static void update_volume(VideoState *is, int sign, double step)
{
    double volume_level = is->audio_volume ? (20 * log(is->audio_volume / (double)SDL_MIX_MAXVOLUME) / log(10)) : -1000.0;
    int new_volume = lrint(SDL_MIX_MAXVOLUME * pow(10.0, (volume_level + sign * step) / 20.0));
    is->audio_volume = av_clip(is->audio_volume == new_volume ? (is->audio_volume + sign) : new_volume, 0, SDL_MIX_MAXVOLUME);
}

//* 在播放器处于暂停状态时，跳转到下一帧。
// 当用户在暂停状态下点击“下一帧”按钮时，调用 step_to_next_frame 函数。
// 在寻址（Seeking）完成后，如果需要立即显示目标帧，也会调用此函数。
static void step_to_next_frame(VideoState* is)
{
    /* if the stream is paused unpause it, then step */
    if (is->paused) // 播放器处于暂停状态（is->paused 为真），则调用 stream_toggle_pause 函数取消暂停。
        stream_toggle_pause(is);
    is->step = 1; // 需要跳转到下一帧。
}

//* 计算视频帧的目标显示延迟
// 根据主时钟和视频时钟的差异，调整视频帧的显示延迟，以实现音视频同步
// double delay: 当前帧的原始显示延迟。
static double compute_target_delay(double delay, VideoState* is)
{
    // 定义 sync_threshold 用于存储同步阈值。
    //定义 diff 用于存储视频时钟和主时钟的差异。
    double sync_threshold, diff = 0;

    /* update delay to follow master synchronisation source */
    // 仅在视频时钟不是主时钟时，才需要调整显示延迟以实现同步。
    if (get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER) {
        /* if video is slave, we try to correct big delays by
           duplicating or deleting a frame */
        diff = get_clock(&is->vidclk) - get_master_clock(is); // 计算视频时钟（is->vidclk）和主时钟（get_master_clock(is)）之间的差异 diff。

        /* skip or repeat frame. We take into account the
           delay to compute the threshold. I still don't know
           if it is the best guess */
        sync_threshold = FFMAX(AV_SYNC_THRESHOLD_MIN, FFMIN(AV_SYNC_THRESHOLD_MAX, delay)); // 同步阈值 sync_threshold 的取值范围为 [AV_SYNC_THRESHOLD_MIN, AV_SYNC_THRESHOLD_MAX]，并且不超过当前延迟 delay。
        //* 调整显示延迟
        // 如果时钟差异 diff 是有效值（!isnan(diff)）且小于最大帧持续时间（is->max_frame_duration），则进行以下调整：
        if (!isnan(diff) && fabs(diff) < is->max_frame_duration) {
            if (diff <= -sync_threshold) // 如果 diff 小于等于 -sync_threshold，则减少延迟（delay = FFMAX(0, delay + diff)）。
                delay = FFMAX(0, delay + diff);
            else if (diff >= sync_threshold && delay > AV_SYNC_FRAMEDUP_THRESHOLD) // 如果 diff 大于等于 sync_threshold 且当前延迟大于 AV_SYNC_FRAMEDUP_THRESHOLD，则增加延迟（delay = delay + diff）。
                delay = delay + diff;
            else if (diff >= sync_threshold) // 如果 diff 大于等于 sync_threshold 但当前延迟较小，则加倍延迟（delay = 2 * delay）。
                delay = 2 * delay;
        }
    }

    av_log(NULL, AV_LOG_TRACE, "video: delay=%0.3f A-V=%f\n",
            delay, -diff);

    return delay;
}

//* 计算两个连续视频帧之间的持续时间
// 根据帧的时间戳（PTS）计算帧之间的时间差，并确保结果合理。
// Frame *vp: 指向当前视频帧的指针。
// Frame *nextvp: 指向下一个视频帧的指针。
static double vp_duration(VideoState* is, Frame* vp, Frame* nextvp)
{
    if (vp->serial == nextvp->serial) { // 检查当前帧 vp 和下一帧 nextvp 的序列号（serial）是否相同。 确保两帧属于同一个序列（即未被重新排序或丢弃）。
        double duration = nextvp->pts - vp->pts; // 计算下一帧的 PTS（nextvp->pts）与当前帧的 PTS（vp->pts）之间的差值，作为帧之间的持续时间。 获取两帧之间的时间间隔。
        // 如果时间差 duration 是无效值（isnan(duration)）、小于等于 0 或超过最大帧持续时间（is->max_frame_duration），则返回当前帧的默认持续时间（vp->duration）。
        // 确保返回的时间差是合理的，避免无效或异常值影响播放。
        if (isnan(duration) || duration <= 0 || duration > is->max_frame_duration)
            return vp->duration;
        else
            return duration;
    } else { //  处理不同序列的帧
        return 0.0; // 如果两帧不属于同一个序列，则认为它们之间没有有效的时间差。
    }
}

//* 更新视频时钟
// 根据当前视频帧的 PTS（Presentation Timestamp）和序列号（serial）更新视频时钟，并将外部时钟（extclk）同步到视频时钟
// double pts: 当前视频帧的 PTS（显示时间戳）。
// int serial: 当前视频帧的序列号。
static void update_video_pts(VideoState* is, double pts, int serial)
{
    /* update current video pts */
    set_clock(&is->vidclk, pts, serial); // 将视频时钟（is->vidclk）更新为当前帧的 PTS 和序列号。
    sync_clock_to_slave(&is->extclk, &is->vidclk); // 将外部时钟（is->extclk）同步到视频时钟（is->vidclk）。 确保外部时钟与视频时钟保持一致，避免时钟漂移。
}

/* called to display each frame */
//* 负责视频帧的刷新和显示。
// 确保视频帧按照正确的时间间隔显示，并且处理音视频同步、帧丢弃、字幕显示等问题。
static void video_refresh(void* opaque, double* remaining_time)
{
    VideoState *is = opaque; 
    double time; // 用于存储当前时间。

    Frame *sp, *sp2; // 用于处理字幕帧的指针。

    //* 1 播放器没有暂停
    //* 2 主同步类型是外部时钟（AV_SYNC_EXTERNAL_CLOCK）
    //* 3 实时流
    //* 则检查外部时钟的速度。
    if (!is->paused && get_master_sync_type(is) == AV_SYNC_EXTERNAL_CLOCK && is->realtime)
        check_external_clock_speed(is);

    //* 非视频的渲染(声音波形等)
    // 1 显示没有被禁用 2 当前显示模式不是视频模式（可能是音频模式或其他模式） 3 存在音频流
    if (!display_disable && is->show_mode != SHOW_MODE_VIDEO && is->audio_st) {
        // 则计算当前时间
        time = av_gettime_relative() / 1000000.0;
        // 如果强制刷新（is->force_refresh）或者距离上次刷新时间已经超过了 rdftspeed
        if (is->force_refresh || is->last_vis_time + rdftspeed < time) {
            // 调用 video_display 函数进行显示，并更新 last_vis_time。
            video_display(is);
            is->last_vis_time = time;
        }
        // 更新 remaining_time，确保不会超过下次刷新的时间。
        *remaining_time = FFMIN(*remaining_time, is->last_vis_time + rdftspeed - time);
    }

    //* 视频帧渲染
    // 存在视频流，则进入视频帧处理逻辑。
    if (is->video_st) {
retry:
        if (frame_queue_nb_remaining(&is->pictq) == 0) {
            // nothing to do, no picture to display in the queue
            //* 帧队列中没有帧，则什么都不做。
        } else {
            //* 否则，从队列中取出当前帧和上一帧。
            double last_duration, duration, delay;
            Frame *vp, *lastvp;

            /* dequeue the picture */
            lastvp = frame_queue_peek_last(&is->pictq);
            vp = frame_queue_peek(&is->pictq);

            //* 如果当前帧的序列号与视频队列的序列号不匹配，则跳过该帧并重试。 可能是seek
            if (vp->serial != is->videoq.serial) {
                frame_queue_next(&is->pictq);
                goto retry;
            }

            //* 如果上一帧和当前帧的序列号不同，则重置帧计时器。
            if (lastvp->serial != vp->serial)
                is->frame_timer = av_gettime_relative() / 1000000.0;

            //* 如果播放器处于暂停状态，则直接跳转到显示部分。
            if (is->paused)
                goto display;

            //* 计算上一帧的持续时间 last_duration 和目标延迟 delay。
            /* compute nominal last_duration */
            last_duration = vp_duration(is, lastvp, vp);
            delay = compute_target_delay(last_duration, is);

            //* 如果当前时间小于帧计时器加上延迟时间，则更新 remaining_time 并跳转到显示部分。
            time = av_gettime_relative() / 1000000.0; 
            if (time < is->frame_timer + delay) { // 解码、渲染未超时，播放
                *remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
                goto display;
            }

            //* 更新帧计时器，并处理同步阈值。
            is->frame_timer += delay;
            if (delay > 0 && time - is->frame_timer > AV_SYNC_THRESHOLD_MAX)
                is->frame_timer = time;

            //* 更新视频的 PTS（Presentation Time Stamp）。
            SDL_LockMutex(is->pictq.mutex);
            if (!isnan(vp->pts))
                update_video_pts(is, vp->pts, vp->serial);
            SDL_UnlockMutex(is->pictq.mutex);

            //* 如果队列中有多个帧，则计算当前帧和下一帧的持续时间，并根据帧丢弃策略决定是否丢弃当前帧。
            if (frame_queue_nb_remaining(&is->pictq) > 1) {
                Frame *nextvp = frame_queue_peek_next(&is->pictq);
                duration = vp_duration(is, vp, nextvp);
                if(!is->step && (framedrop>0 || (framedrop && get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER)) && time > is->frame_timer + duration){
                    is->frame_drops_late++;
                    frame_queue_next(&is->pictq);
                    goto retry;
                }
            }

            //* 对字幕的处理
            // 如果存在字幕流，则处理字幕帧。
            if (is->subtitle_st) {
                // 如果字幕队列 is->subpq 中有字幕帧，则获取当前字幕帧 sp。
                while (frame_queue_nb_remaining(&is->subpq) > 0) {
                    sp = frame_queue_peek(&is->subpq);

                    // 如果字幕队列中有多个字幕帧，则获取下一帧字幕 sp2。  用于检查当前字幕帧和下一帧字幕的时间戳。
                    if (frame_queue_nb_remaining(&is->subpq) > 1)
                        sp2 = frame_queue_peek_next(&is->subpq);
                    else
                        sp2 = NULL;

                    // 判断当前字幕帧是否需要显示或清理
                    if (sp->serial != is->subtitleq.serial // 如果当前字幕帧的序列号与字幕队列的序列号不匹配
                            || (is->vidclk.pts > (sp->pts + ((float) sp->sub.end_display_time / 1000))) // 当前视频帧的时间戳（is->vidclk.pts）超过了当前字幕帧的结束显示时间（sp->pts + sp->sub.end_display_time / 1000）
                            || (sp2 && is->vidclk.pts > (sp2->pts + ((float) sp2->sub.start_display_time / 1000)))) // 存在下一帧字幕（sp2）且当前视频帧的时间戳超过了下一帧字幕的开始显示时间（sp2->pts + sp2->sub.start_display_time / 1000）
                    { // 清理过期的字幕帧
                        if (sp->uploaded) { // 如果当前字幕帧已经上传到纹理（sp->uploaded == 1），则清理字幕纹理：  清理过期的字幕帧，避免残留的字幕影响显示。
                            int i;
                            for (i = 0; i < sp->sub.num_rects; i++) { // 遍历字幕帧中的所有矩形（sp->sub.num_rects）。
                                AVSubtitleRect *sub_rect = sp->sub.rects[i];
                                uint8_t *pixels;
                                int pitch, j;
                                // 锁定字幕纹理（SDL_LockTexture），获取像素数据指针 pixels 和行间距 pitch。
                                if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)&pixels, &pitch)) {
                                    for (j = 0; j < sub_rect->h; j++, pixels += pitch)
                                        memset(pixels, 0, sub_rect->w << 2); // 使用 memset 将字幕矩形的像素数据填充为 0（透明黑色）。
                                    SDL_UnlockTexture(is->sub_texture); // 解锁字幕纹理（SDL_UnlockTexture）。
                                }
                            }
                        }
                        frame_queue_next(&is->subpq); // 将当前字幕帧从队列中移除。
                    } else { // 如果当前字幕帧需要显示，则退出循环。
                        break;
                    }
                }
            }

            //* 显示帧
            // 移动到下一帧，并设置强制刷新标志。
            frame_queue_next(&is->pictq);
            is->force_refresh = 1;
            // 如果处于单步模式且未暂停，则切换暂停状态。
            if (is->step && !is->paused)
                stream_toggle_pause(is);
        }
    display:
        //* 显示图片
        /* display picture */
        //* 如果显示没有被禁用，并且需要强制刷新，并且当前显示模式是视频模式，并且帧队列中有帧可以显示，则调用 video_display 函数显示帧。
        if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown)
            video_display(is);
    }
    //*  结束, 重置强制刷新标志
    is->force_refresh = 0;
    //* console 状态显示
    // 如果需要显示状态信息，则计算并打印当前播放状态，包括主时钟、音视频同步差异、帧丢弃数量、音频队列大小、视频队列大小、字幕队列大小等信息。
    if (show_status) {
        AVBPrint buf;
        static int64_t last_time;
        int64_t cur_time;
        int aqsize, vqsize, sqsize;
        double av_diff;

        cur_time = av_gettime_relative();
        if (!last_time || (cur_time - last_time) >= 30000) {
            aqsize = 0;
            vqsize = 0;
            sqsize = 0;
            if (is->audio_st)
                aqsize = is->audioq.size;
            if (is->video_st)
                vqsize = is->videoq.size;
            if (is->subtitle_st)
                sqsize = is->subtitleq.size;
            av_diff = 0;
            if (is->audio_st && is->video_st)
                av_diff = get_clock(&is->audclk) - get_clock(&is->vidclk);
            else if (is->video_st)
                av_diff = get_master_clock(is) - get_clock(&is->vidclk);
            else if (is->audio_st)
                av_diff = get_master_clock(is) - get_clock(&is->audclk);

            av_bprint_init(&buf, 0, AV_BPRINT_SIZE_AUTOMATIC);
            av_bprintf(&buf,
                      "%7.2f %s:%7.3f fd=%4d aq=%5dKB vq=%5dKB sq=%5dB \r",
                      get_master_clock(is),
                      (is->audio_st && is->video_st) ? "A-V" : (is->video_st ? "M-V" : (is->audio_st ? "M-A" : "   ")),
                      av_diff,
                      is->frame_drops_early + is->frame_drops_late,
                      aqsize / 1024,
                      vqsize / 1024,
                      sqsize);

            if (show_status == 1 && AV_LOG_INFO > av_log_get_level())
                fprintf(stderr, "%s", buf.str);
            else
                av_log(NULL, AV_LOG_INFO, "%s", buf.str);

            fflush(stderr);
            av_bprint_finalize(&buf, NULL);

            last_time = cur_time;
        }
    }
}

//* 将解码后的视频帧（AVFrame）加入帧队列（frame_queue）
static int queue_picture(VideoState *is, AVFrame *src_frame, double pts, double duration, int64_t pos, int serial)
{
    Frame *vp;

#if defined(DEBUG_SYNC)
    printf("frame_type=%c pts=%0.3f\n",
           av_get_picture_type_char(src_frame->pict_type), pts);
#endif

    //* 从帧队列中获取一个可写的帧（Frame 结构体）。
    if (!(vp = frame_queue_peek_writable(&is->pictq)))
        return -1;

    //* 帧信息复制
    vp->sar = src_frame->sample_aspect_ratio;
    vp->uploaded = 0;

    vp->width = src_frame->width;
    vp->height = src_frame->height;
    vp->format = src_frame->format;

    vp->pts = pts;
    vp->duration = duration;
    vp->pos = pos;
    vp->serial = serial;

    //* 设置默认窗口大小
    set_default_window_size(vp->width, vp->height, vp->sar);

    //* 将 src_frame 的引用移动到 vp->frame，避免内存拷贝。
    av_frame_move_ref(vp->frame, src_frame);
    //* 将帧加入队列。
    frame_queue_push(&is->pictq);
    return 0;
}

//* 从视频解码器获取解码后的视频帧。
static int get_video_frame(VideoState *is, AVFrame *frame)
{
    int got_picture;

    //*  解码视频帧
    // got_picture > 0：成功解码到一帧视频。
    // got_picture == 0：没有解码到视频帧。
    // got_picture < 0：解码出错。
    if ((got_picture = decoder_decode_frame(&is->viddec, frame, NULL)) < 0)
        return -1; //解码出错，函数返回 -1

    if (got_picture) {
        //  计算帧的显示时间戳（PTS）
        double dpts = NAN;

        if (frame->pts != AV_NOPTS_VALUE)
            dpts = av_q2d(is->video_st->time_base) * frame->pts;

        // 计算帧的宽高比
        frame->sample_aspect_ratio = av_guess_sample_aspect_ratio(is->ic, is->video_st, frame);

        //* 如果启用了帧丢弃（framedrop），则根据帧的显示时间戳（PTS）和主时钟的同步情况，决定是否丢弃该帧。
        if (framedrop > 0 || // 强制启用帧丢弃。
            (framedrop && get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER)) { // 如果启用了帧丢弃，并且主时钟不是视频时钟（AV_SYNC_VIDEO_MASTER），则启用帧丢弃。
            if (frame->pts != AV_NOPTS_VALUE) {
                double diff = dpts - get_master_clock(is); // 帧的显示时间戳与主时钟的差值。
                if (!isnan(diff) // 时间差有效（不是 NAN）。
                    && fabs(diff) < AV_NOSYNC_THRESHOLD && // 时间差的绝对值小于同步阈值
                    diff - is->frame_last_filter_delay < 0 && // 帧的显示时间早于主时钟（考虑滤镜延迟）。
                    is->viddec.pkt_serial == is->vidclk.serial && // 视频解码器的序列号与视频时钟的序列号一致。
                    is->videoq.nb_packets) { // 视频队列中还有未处理的数据包。
                    // 如果满足丢弃条件，则增加 frame_drops_early 计数器，释放帧的引用（av_frame_unref(frame)），并将 got_picture 设置为 0。
                    is->frame_drops_early++;   
                    av_frame_unref(frame);
                    got_picture = 0;
                }
            }
        }
    }

    return got_picture;
}

//* 配置一个 FFmpeg 的滤镜图, 根据输入的滤镜图描述字符串（filtergraph）将源滤镜（source_ctx）和接收滤镜（sink_ctx）连接起来
// graph: 指向 AVFilterGraph 结构的指针，表示要配置的滤镜图。
// filtergraph: 一个字符串，描述了滤镜图的连接方式。
// source_ctx: 指向源滤镜（AVFilterContext）的指针，通常是输入滤镜。
// sink_ctx: 指向接收滤镜（AVFilterContext）的指针，通常是输出滤镜。
static int configure_filtergraph(AVFilterGraph* graph, const char* filtergraph,
    AVFilterContext* source_ctx, AVFilterContext* sink_ctx)
{
    int ret, i;
    int nb_filters = graph->nb_filters; // 存储滤镜图中滤镜的数量。
    AVFilterInOut *outputs = NULL, *inputs = NULL; // 用于存储滤镜图的输入和输出连接点的指针。

    //* 判断是否指定过滤器图字符串，创建不同的过滤器链
    if (filtergraph) {
        //* 分配输入输出结构
        outputs = avfilter_inout_alloc();
        inputs  = avfilter_inout_alloc();
        if (!outputs || !inputs) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        //* 配置输出连接点
        outputs->name = av_strdup("in"); // 设置为 "in"，表示输入端的名称。
        outputs->filter_ctx = source_ctx;   // 设置为 source_ctx，表示源滤镜。
        outputs->pad_idx    = 0;    // 设置为 0，表示使用源滤镜的第一个输出 pad。
        outputs->next       = NULL; // 设置为 NULL，表示没有下一个输出连接点。
        //* 配置输入连接点
        inputs->name        = av_strdup("out"); // 设置为 "out"，表示输出端的名称。
        inputs->filter_ctx  = sink_ctx; // 设置为 sink_ctx，表示接收滤镜。
        inputs->pad_idx     = 0;        //  设置为 0，表示使用接收滤镜的第一个输入 pad。
        inputs->next        = NULL; //  设置为 NULL，表示没有下一个输入连接点。

        //* 解析滤镜图描述字符串
        if ((ret = avfilter_graph_parse_ptr(graph, filtergraph, &inputs, &outputs, NULL)) < 0)
            goto fail;
    } else {
        //* 直接连接源滤镜和接收滤镜
        if ((ret = avfilter_link(source_ctx, 0, sink_ctx, 0)) < 0)
            goto fail;
    }

    /* Reorder the filters to ensure that inputs of the custom filters are merged first */
    //* 重新排序滤镜图中的滤镜，确保自定义滤镜的输入先被合并。
    for (i = 0; i < graph->nb_filters - nb_filters; i++)
        FFSWAP(AVFilterContext*, graph->filters[i], graph->filters[i + nb_filters]);

    //* 配置滤镜图，使其准备好处理数据。
    ret = avfilter_graph_config(graph, NULL);
fail:
    avfilter_inout_free(&outputs);
    avfilter_inout_free(&inputs);
    return ret;
}

//* 根据视频帧的属性（如宽度、高度、像素格式等）和用户指定的滤镜参数，创建并配置一个滤镜图（AVFilterGraph），以便对视频帧进行处理。
static int configure_video_filters(AVFilterGraph *graph, VideoState *is, const char *vfilters, AVFrame *frame)
{
    enum AVPixelFormat pix_fmts[FF_ARRAY_ELEMS(sdl_texture_format_map)]; // ffmpeg支持的滤镜像素格式列表。
    char sws_flags_str[512] = ""; // 用于存储 sws_flags 的字符串。
    char buffersrc_args[256]; // 用于存储 buffer 滤镜参数的字符串。
    int ret; 
    AVFilterContext* filt_src = NULL, * filt_out = NULL, * last_filter = NULL; // 滤镜图的输入、输出和滤镜图中的最后一个滤镜。
        // 这里的最后一个，不是链上最后一个，而是最后一个添加进滤镜图中的最后一个
    AVCodecParameters* codecpar = is->video_st->codecpar; // 视频流的编解码参数。
    AVRational fr = av_guess_frame_rate(is->ic, is->video_st, NULL); // 视频流的帧率。
    const AVDictionaryEntry *e = NULL; 
    int nb_pix_fmts = 0;
    int i, j;
    AVBufferSrcParameters *par = av_buffersrc_parameters_alloc(); // buffer 滤镜的参数。

    if (!par)
        return AVERROR(ENOMEM);

    //* 根据渲染器支持的纹理格式，填充一个 FFmpeg 像素格式数组（pix_fmts） sdl->ffmpeg
    // 在播放器的视频处理线程中，通常会根据渲染器支持的纹理格式，动态配置滤镜图的输出像素格式。
    // 通过填充 pix_fmts 数组，可以确保滤镜图的输出格式与渲染器兼容。
    for (i = 0; i < renderer_info.num_texture_formats; i++) { //  renderer_info.num_texture_formats：渲染器支持的纹理格式数量。
        for (j = 0; j < FF_ARRAY_ELEMS(sdl_texture_format_map) - 1; j++) {
            // 根据渲染器支持的纹理格式（renderer_info.texture_formats），填充 pix_fmts 数组。
            if (renderer_info.texture_formats[i] == sdl_texture_format_map[j].texture_fmt) { // renderer_info.texture_formats[i]：渲染器支持的第 i 个纹理格式。
                pix_fmts[nb_pix_fmts++] = sdl_texture_format_map[j].format;
                break;
            }
        }
    }
    // pix_fmts 数组以 AV_PIX_FMT_NONE 结尾。
    pix_fmts[nb_pix_fmts] = AV_PIX_FMT_NONE; 

    //* 配置 sws_flags
    // 从 sws_dict 字典中提取 sws_flags 和其他相关参数，并将其格式化为一个字符串（sws_flags_str）
    while ((e = av_dict_iterate(sws_dict, e))) { // 遍历字典中的键值对。
        if (!strcmp(e->key, "sws_flags")) { // 一个特殊的键，用于指定缩放滤镜（swscale）的标志。
            av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", "flags", e->value); // 当前键是 sws_flags，则将其值格式化为 flags=<value>:，并追加到 sws_flags_str 中。
        } else // 非 sws_flags 的键，将其键值对格式化为 <key>=<value>:，并追加到 sws_flags_str 中。
            av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", e->key, e->value);
    }
    // 如果 sws_flags_str 不为空，则将其最后一个字符（分号 :）替换为字符串结束符 \0。
    if (strlen(sws_flags_str))
        sws_flags_str[strlen(sws_flags_str)-1] = '\0';
    // 将该字符串设置为滤镜图（AVFilterGraph）的 scale_sws_opts 属性。
    graph->scale_sws_opts = av_strdup(sws_flags_str);

    //* 生成一个字符串（buffersrc_args），用于配置 buffer 滤镜的参数。
    // 根据视频帧的属性（宽度、高度、像素格式等）和视频流的参数（时间基、宽高比等），生成 buffer 滤镜的参数字符串。
    snprintf(buffersrc_args, sizeof(buffersrc_args),
             "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d:" // video_size=%dx%d：视频帧的宽度和高度。
             "colorspace=%d:range=%d", // pix_fmt=%d：视频帧的像素格式。 // time_base=%d/%d：视频流的时间基（time_base）。 // pixel_aspect=%d/%d：视频帧的宽高比（sample_aspect_ratio）。
             frame->width, frame->height, frame->format, // colorspace=%d：视频帧的颜色空间。 // range=%d：视频帧的颜色范围。
             is->video_st->time_base.num, is->video_st->time_base.den,
             codecpar->sample_aspect_ratio.num, FFMAX(codecpar->sample_aspect_ratio.den, 1), // FFMAX(codecpar->sample_aspect_ratio.den, 1)：确保分母不为 0。
        frame->colorspace, frame->color_range);
    // 如果视频流的帧率有效，则将帧率信息追加到参数字符串中。
    if (fr.num && fr.den)
        av_strlcatf(buffersrc_args, sizeof(buffersrc_args), ":frame_rate=%d/%d", fr.num, fr.den);

    //*  创建 buffer 滤镜
    // buffer 滤镜是 FFmpeg 滤镜图中的输入滤镜，用于将原始视频帧数据输入到滤镜图中进行处理。
    if ((ret = avfilter_graph_create_filter(&filt_src, // 指向滤镜上下文的指针，用于存储创建的滤镜。
                                            avfilter_get_by_name("buffer"), // 获取 buffer 滤镜的定义。
                                            "ffplay_buffer", buffersrc_args, NULL, // 滤镜的名称。uffer 滤镜的参数（如视频帧的宽度、高度、像素格式等）。滤镜的私有数据（此处为 NULL）。
                                            graph)) < 0) // 滤镜图（AVFilterGraph）。
        goto fail;
    par->hw_frames_ctx = frame->hw_frames_ctx; // 如果视频帧使用硬件加速（如 GPU 解码），则设置硬件帧上下文。
    // 设置 buffer 滤镜的参数（par）。
    ret = av_buffersrc_parameters_set(filt_src, par);
    if (ret < 0)
        goto fail;

    //* 创建 buffersink 滤镜
    // buffersink 滤镜：是滤镜图的输出滤镜，用于从滤镜图中获取处理后的视频帧数据。需要指定输出帧的像素格式和颜色空间。
    ret = avfilter_graph_create_filter(&filt_out, // 指向滤镜上下文的指针，用于存储创建的滤镜。
                                       avfilter_get_by_name("buffersink"), // 获取 buffersink 滤镜的定义。
                                       "ffplay_buffersink", NULL, NULL, graph); // 滤镜的名称。滤镜的参数（此处为 NULL）。滤镜的私有数据（此处为 NULL）。滤镜图（AVFilterGraph）。
    if (ret < 0)
        goto fail;
    // 设置 buffersink 滤镜的像素格式和颜色空间。 // "pix_fmts" 选项名称，表示像素格式列表。  // "color_spaces"：选项名称，表示颜色空间列表。
    if ((ret = av_opt_set_int_list(filt_out, "pix_fmts", pix_fmts,  AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto fail;
    if (!vk_renderer &&
        (ret = av_opt_set_int_list(filt_out, "color_spaces", sdl_supported_color_spaces,  AVCOL_SPC_UNSPECIFIED, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto fail;

    last_filter = filt_out;

/* Note: this macro adds a filter before the lastly added filter, so the
 * processing order of the filters is in reverse */
//* 在滤镜图最后一个加入的滤镜之前插入一个新的滤镜。
#define  INSERT_FILT(name, arg) do {                                          \
    AVFilterContext *filt_ctx;                                               \
                                                                             \
    ret = avfilter_graph_create_filter(&filt_ctx,                            \ // 创建滤镜并将其添加到滤镜图中。
                                       avfilter_get_by_name(name),           \ // 获取指定名称的滤镜定义。
                                       "ffplay_" name, arg, NULL, graph);    \ // "ffplay_" name：滤镜的名称（添加前缀 "ffplay_" 以避免名称冲突）。
    if (ret < 0)                                                             \
        goto fail;                                                           \
                                                                             \
    ret = avfilter_link(filt_ctx, 0, last_filter, 0);                        \ // 连接两个滤镜。
    if (ret < 0)                                                             \
        goto fail;                                                           \
                                                                             \
    last_filter = filt_ctx;                                                  \ // 更新 last_filter
} while (0)
    //* 滤镜图 ： buffer -> ...... -> last_filter -> ... -> buffersink  也就是一开始只有buffer/buffersink，然后


    //* 自动旋转功能
    // 根据视频流中的旋转信息（通常由拍摄设备写入），自动调整视频帧的方向，确保视频播放时画面是正立的。这在处理手机拍摄的视频时尤其有用，因为手机拍摄的视频通常会包含旋转元数据。
    if (autorotate) {
        double theta = 0.0;
        int32_t *displaymatrix = NULL;
        //* 尝试从当前帧的附加数据（AVFrameSideData）中获取旋转矩阵。
        AVFrameSideData *sd = av_frame_get_side_data(frame, AV_FRAME_DATA_DISPLAYMATRIX);
        if (sd)
            displaymatrix = (int32_t*)sd->data;
        //* 如果帧中没有旋转矩阵，则尝试从视频流的编码参数（codecpar）中的数据包附加数据（AVPacketSideData）中获取。
        if (!displaymatrix) {
            const AVPacketSideData *psd = av_packet_side_data_get(is->video_st->codecpar->coded_side_data,
                                                                  is->video_st->codecpar->nb_coded_side_data,
                                                                  AV_PKT_DATA_DISPLAYMATRIX);
            if (psd)
                displaymatrix = (int32_t *)psd->data;
        }
        //* 从旋转矩阵中计算出旋转角度 theta。
        theta = get_rotation(displaymatrix);
        //* 根据旋转角度插入滤镜：
        // 如果旋转角度接近 90度，插入 transpose 滤镜，并设置参数为 clock（顺时针旋转）。
        // 如果旋转角度接近 180度，依次插入 hflip（水平翻转）和 vflip（垂直翻转）滤镜。
        // 如果旋转角度接近 270度，插入 transpose 滤镜，并设置参数为 cclock（逆时针旋转）。
        // 如果旋转角度是其他值，则通过 rotate 滤镜实现任意角度的旋转，角度值通过公式 theta * PI / 180 计算。
        if (fabs(theta - 90) < 1.0) {
            INSERT_FILT("transpose", "clock");
        } else if (fabs(theta - 180) < 1.0) {
            INSERT_FILT("hflip", NULL);
            INSERT_FILT("vflip", NULL);
        } else if (fabs(theta - 270) < 1.0) {
            INSERT_FILT("transpose", "cclock");
        } else if (fabs(theta) > 1.0) {
            char rotate_buf[64];
            snprintf(rotate_buf, sizeof(rotate_buf), "%f*PI/180", theta);
            INSERT_FILT("rotate", rotate_buf);
        }
    }

    //*  配置滤镜图
    if ((ret = configure_filtergraph(graph, vfilters, filt_src, last_filter)) < 0)
        goto fail;

    //* 将配置好的滤镜图保存到 VideoState 中。
    is->in_video_filter = filt_src;
    is->out_video_filter = filt_out;

fail:
    av_freep(&par);
    return ret;
}

//* 根据输入的音频参数和用户指定的过滤器字符串，构建一个音频过滤器链，并将输入音频流转换为目标格式。
// VideoState *is：ffplay 的全局状态对象，包含音频、视频和字幕的相关信息。
// const char *afilters：用户指定的音频过滤器字符串（如重采样、音量调整等）。
// int force_output_format：是否强制将输出格式设置为目标格式。
static int configure_audio_filters(VideoState* is, const char* afilters, int force_output_format)
{
    static const enum AVSampleFormat sample_fmts[] = { AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_NONE }; // 支持的音频采样格式列表，以 AV_SAMPLE_FMT_NONE 结尾。
    int sample_rates[2] = { 0, -1 };    // 支持的采样率列表，以 -1 结尾。
    AVFilterContext *filt_asrc = NULL, *filt_asink = NULL; // 分别指向过滤器链的输入和输出端。
    char aresample_swr_opts[512] = "";   // 用于存储重采样选项的字符串。
    const AVDictionaryEntry *e = NULL;  // 用于遍历字典的临时变量。
    AVBPrint bp;                        // 用于动态构建字符串的 AVBPrint 对象。
    char asrc_args[256];                // 用于存储输入过滤器参数的字符串。
    int ret;

    //* 释放旧的过滤器图并创建新的过滤器图
    avfilter_graph_free(&is->agraph);
    if (!(is->agraph = avfilter_graph_alloc()))
        return AVERROR(ENOMEM);
    is->agraph->nb_threads = filter_nbthreads; // 设置过滤器图的线程数

    //* 初始化动态字符串builder
    av_bprint_init(&bp, 0, AV_BPRINT_SIZE_AUTOMATIC);

    //* 构建重采样选型字符串
    // 将重采样选项从字典转换为字符串，并设置到过滤器图中。
    while ((e = av_dict_iterate(swr_opts, e)))
        av_strlcatf(aresample_swr_opts, sizeof(aresample_swr_opts), "%s=%s:", e->key, e->value); // 将字典中的键值对拼接到字符串中。
    if (strlen(aresample_swr_opts)) 
        aresample_swr_opts[strlen(aresample_swr_opts)-1] = '\0';
    av_opt_set(is->agraph, "aresample_swr_opts", aresample_swr_opts, 0); // 将重采样选项字符串设置到过滤器图中。

    //* 描述声道布局
    av_channel_layout_describe_bprint(&is->audio_filter_src.ch_layout, &bp);

    //* 构建输入过滤器参数
    // sample_rate：采样率。
    // sample_fmt：采样格式。
    // time_base：时间基（1/采样率）。
    // channel_layout：声道布局。
    ret = snprintf(asrc_args, sizeof(asrc_args),
                   "sample_rate=%d:sample_fmt=%s:time_base=%d/%d:channel_layout=%s",
                   is->audio_filter_src.freq, av_get_sample_fmt_name(is->audio_filter_src.fmt),
                   1, is->audio_filter_src.freq, bp.str);

    //* 创建输入过滤器 abuffer
    ret = avfilter_graph_create_filter(&filt_asrc,
                                       avfilter_get_by_name("abuffer"), "ffplay_abuffer", // abuffer：输入过滤器，用于将原始音频数据送入过滤器链。
                                       asrc_args, NULL, is->agraph);
    if (ret < 0)
        goto end;

    //* 创建输出过滤器
    // 创建输出过滤器（abuffersink），用于接收处理后的音频数据。
    ret = avfilter_graph_create_filter(&filt_asink,
                                       avfilter_get_by_name("abuffersink"), "ffplay_abuffersink", // abuffersink：输出过滤器，用于从过滤器链中获取处理后的音频数据。
                                       NULL, NULL, is->agraph);
    if (ret < 0)
        goto end;

    //* 设置输出过滤器的采样格式和声道数
    // 设置支持的采样格式列表。
    if ((ret = av_opt_set_int_list(filt_asink, "sample_fmts", sample_fmts, AV_SAMPLE_FMT_NONE, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto end;
    // 设置是否支持所有声道数。
    if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 1, AV_OPT_SEARCH_CHILDREN)) < 0)
        goto end;

    //* 强制设置输出格式
    if (force_output_format) {
        av_bprint_clear(&bp);
        // 在stream_component_open中force_output_format设置为0，audio_tgt.ch_layout未被设置
        // 在stream_component_open的audio_open中，audio_tgt.ch_layout被设置
        // 然后在audio_thread中force_output_format被设置为1，重入本函数，此时取到有效的ch_layout
            av_channel_layout_describe_bprint(&is->audio_tgt.ch_layout, &bp);
            sample_rates[0] = is->audio_tgt.freq;
        if ((ret = av_opt_set_int(filt_asink, "all_channel_counts", 0, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
        if ((ret = av_opt_set(filt_asink, "ch_layouts", bp.str, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
        if ((ret = av_opt_set_int_list(filt_asink, "sample_rates"   , sample_rates   ,  -1, AV_OPT_SEARCH_CHILDREN)) < 0)
            goto end;
    }

    //*  配置过滤器链
    if ((ret = configure_filtergraph(is->agraph, afilters, filt_asrc, filt_asink)) < 0)
        goto end;

    //* 保存输入和输出过滤器
    is->in_audio_filter = filt_asrc;
    is->out_audio_filter = filt_asink;

end:
    if (ret < 0)
        avfilter_graph_free(&is->agraph);
    av_bprint_finalize(&bp, NULL);

    return ret;
}

//* 负责音频解码和音频帧的处理
// 运行在一个独立的线程中，负责从音频流中解码音频帧，并将解码后的音频帧放入音频帧队列中，供音频播放线程使用。
static int audio_thread(void* arg)
{
    VideoState *is = arg;
    AVFrame *frame = av_frame_alloc(); // 存储解码后的音频帧。
    Frame *af; // 指向音频帧队列中的一个帧。
    int last_serial = -1;   // 记录上一次处理的音频包的序列号，用于检测音频流是否发生了变化。
    int reconfigure;    // 标记是否需要重新配置音频过滤器。
    int got_frame = 0;  // 标记是否成功解码了一帧音频。
    AVRational tb;  // 存储时间基（time base），表示音频帧的时间戳单位。
    int ret = 0;  // 存储函数调用的返回值。

    //* 如果分配失败，返回 ENOMEM 错误。
    if (!frame)
        return AVERROR(ENOMEM);

    //* 进入主循环， ret < 0时产生错误，退出音频解码线程
    do {
        // 从音频解码器中解码一帧音频数据，解码失败退出
        if ((got_frame = decoder_decode_frame(&is->auddec, frame, NULL)) < 0)
            goto the_end;

        if (got_frame) { // 成功解码了一帧音频
            tb = (AVRational){1, frame->sample_rate}; // 设置时间基 tb 为 {1, frame->sample_rate}，即每秒钟的样本数。

            //* 检查是否需要重新配置音频过滤器
            reconfigure =  // 如果音频格式、声道布局、采样率或序列号发生了变化，则需要重新配置音频过滤器。
                cmp_audio_fmts(is->audio_filter_src.fmt, is->audio_filter_src.ch_layout.nb_channels,
                            frame->format, frame->ch_layout.nb_channels)    ||
                av_channel_layout_compare(&is->audio_filter_src.ch_layout, &frame->ch_layout) ||
                is->audio_filter_src.freq           != frame->sample_rate ||
                is->auddec.pkt_serial               != last_serial;

            if (reconfigure) {
                // 记录音频帧的格式变化，并通过日志输出这些变化的具体信息。
                char buf1[1024], buf2[1024];   // 将音频声道布局（AVChannelLayout）转换为可读的字符串描述。
                av_channel_layout_describe(&is->audio_filter_src.ch_layout, buf1, sizeof(buf1));
                av_channel_layout_describe(&frame->ch_layout, buf2, sizeof(buf2));
                av_log(NULL, AV_LOG_DEBUG,
                    "Audio frame changed from rate:%d ch:%d fmt:%s layout:%s serial:%d to rate:%d ch:%d fmt:%s layout:%s serial:%d\n",
                    is->audio_filter_src.freq, is->audio_filter_src.ch_layout.nb_channels, av_get_sample_fmt_name(is->audio_filter_src.fmt), buf1, last_serial,
                    frame->sample_rate, frame->ch_layout.nb_channels, av_get_sample_fmt_name(frame->format), buf2, is->auddec.pkt_serial);

                // 更新音频过滤器源（audio_filter_src）的配置，以匹配新解码的音频帧（frame）的格式、声道布局和采样率。
                is->audio_filter_src.fmt = frame->format;
                ret = av_channel_layout_copy(&is->audio_filter_src.ch_layout, &frame->ch_layout);
                if (ret < 0)
                    goto the_end;
                is->audio_filter_src.freq           = frame->sample_rate;
                last_serial                         = is->auddec.pkt_serial;

                // 调用 configure_audio_filters 函数重新配置音频过滤器。
                if ((ret = configure_audio_filters(is, afilters, 1)) < 0)
                    goto the_end;
            }

            //* 将解码后的音频帧（frame）添加到音频过滤器图的输入缓冲区（is->in_audio_filter）中，以便后续通过过滤器图对音频帧进行处理。
            if ((ret = av_buffersrc_add_frame(is->in_audio_filter, frame)) < 0)
                goto the_end;

            //* 从音频过滤器图的输出缓冲区（is->out_audio_filter）中获取处理后的音频帧，并将其放入音频帧队列（is->sampq）中，供音频播放线程使用。
            while ((ret = av_buffersink_get_frame_flags(is->out_audio_filter, frame, 0)) >= 0) { // 从过滤器图的输出缓冲区中获取处理后的音频帧。
                // 获取帧的附加信息
                // frame->opaque_ref：是一个指向附加数据的引用，通常用于存储与帧相关的元数据（如包位置 pkt_pos）。
                FrameData* fd = frame->opaque_ref ? (FrameData*)frame->opaque_ref->data : NULL; // 存在，则将其转换为 FrameData 结构体指针（fd）。
                tb = av_buffersink_get_time_base(is->out_audio_filter); // 获取过滤器图输出缓冲区的时间基（tb），用于将帧的时间戳（pts）转换为秒。

                //* 获取可写的音频帧队列项
                if (!(af = frame_queue_peek_writable(&is->sampq)))
                    goto the_end;
                //* 向帧队列中的帧写数据
                // 设置帧的属性
                af->pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
                    // 设置帧的显示时间戳（PTS）。
                    // 如果 frame->pts 为 AV_NOPTS_VALUE（表示无有效时间戳），则将 af->pts 设置为 NAN。
                    // 否则，将 frame->pts 乘以时间基 tb，将其转换为秒。
                af->pos = fd ? fd->pkt_pos : -1; // 设置帧的包位置（pkt_pos）。
                af->serial = is->auddec.pkt_serial; // 设置帧的序列号（serial），用于标识帧所属的音频流。
                af->duration = av_q2d((AVRational) { frame->nb_samples, frame->sample_rate });
                    // 设置帧的持续时间。 通过 frame->nb_samples（帧中的样本数）和 frame->sample_rate（采样率）计算帧的持续时间，并将其转换为秒。
                // 将帧移动到音频帧队列
                av_frame_move_ref(af->frame, frame); // 将 frame 的内容移动到 af->frame 中，并清空 frame。 这样可以避免复制帧数据，提高效率。
                //* 推帧入队（改索引）
                frame_queue_push(&is->sampq);

                //* 检查序列号， 当音频流的序列号发生变化时（如 seek 操作），线程会退出当前循环。
                if (is->audioq.serial != is->auddec.pkt_serial)
                    break;
                // is->audioq.serial：表示音频队列的序列号。
                // is->auddec.pkt_serial：表示当前音频解码器的包序列号。
                // 如果两者不一致，说明音频流发生了变化（如发生了 seek 操作），此时退出循环。
            }
            //* 处理 EOF 情况
            if (ret == AVERROR_EOF) // 如果 av_buffersink_get_frame_flags 返回 AVERROR_EOF，表示过滤器图的输出缓冲区已经结束。
                is->auddec.finished = is->auddec.pkt_serial; // 将音频解码器的 finished 标志设置为当前包的序列号，表示解码器已经完成。
        }
    } while (ret >= 0 || ret == AVERROR(EAGAIN) || ret == AVERROR_EOF); 
    // ret == AVERROR_EOF：
    // 如果上一次操作返回 AVERROR_EOF，表示已经到达流的末尾。
    // 此时，循环会继续，以便处理剩余的帧或完成清理工作。
    // ret < 0  ==> goto end || decoder_decode_frame < 0 
    
the_end:
// 1. decoder_decode_frame 解码失败，返回负值，退出
// 2. decoder_decode_frame 中abort_request被设置，返回-1， 产生中断请求，退出
// 3. decoder_decode_frame 产生AVERROR_EOF，表示到达流末尾，流结束，退出
// 4. 音频帧队列已满，无法写入，退出
// 5. 过滤器图操作错误，退出
// 6. 音频流序列号发生变化，退出
    avfilter_graph_free(&is->agraph);
    av_frame_free(&frame);
    return ret;
}

static int decoder_start(Decoder *d, int (*fn)(void *), const char *thread_name, void* arg)
{
    packet_queue_start(d->queue);
    d->decoder_tid = SDL_CreateThread(fn, thread_name, arg);
    if (!d->decoder_tid) {
        av_log(NULL, AV_LOG_ERROR, "SDL_CreateThread(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    return 0;
}

//* 从视频解码器获取解码后的视频帧，应用视频滤镜（如果存在），并将处理后的帧放入视频帧队列中，供渲染线程使用。
// 1 从视频解码器获取解码后的视频帧。
// 2 应用视频滤镜（如果配置了滤镜）。
// 3 将处理后的视频帧放入视频帧队列中，供渲染线程使用。
static int video_thread(void* arg)
{
    VideoState *is = arg;
    AVFrame *frame = av_frame_alloc(); // 用于存储解码后的视频帧。
    double pts;
    double duration;
    int ret;
    AVRational tb = is->video_st->time_base; // 视频流的时间基（time_base）
    AVRational frame_rate = av_guess_frame_rate(is->ic, is->video_st, NULL); // 视频流的帧率。

    AVFilterGraph *graph = NULL; //用于视频滤镜的滤镜图和
    AVFilterContext* filt_out = NULL, * filt_in = NULL; // 滤镜输入输出端
    //* last 缓存上一次视频帧的属性，以便检测视频帧属性是否发生变化。
    int last_w = 0;
    int last_h = 0;
    enum AVPixelFormat last_format = -2;
    int last_serial = -1;
    int last_vfilter_idx = 0;

    if (!frame)
        return AVERROR(ENOMEM);

    for (;;) {
        //* 从视频解码器获取解码后的视频帧。
        ret = get_video_frame(is, frame);
        if (ret < 0)
            goto the_end; // 出错，跳转到 the_end 进行清理。
        if (!ret)
            continue; // 没有获取到帧，继续循环。

        //* 动态重新配置视频滤镜
        if (last_w != frame->width // 如果视频帧的宽度
            || last_h != frame->height // 、高度
            || last_format != frame->format //、像素格式
            || last_serial != is->viddec.pkt_serial // 、序列号
            || last_vfilter_idx != is->vfilter_idx) { // 或滤镜索引发生变化，则需要重新配置视频滤镜。以确保滤镜能够正确处理新的视频帧。
            // 打印视频帧属性变化前后的详细信息，包括宽度、高度、像素格式和序列号。
            av_log(NULL, AV_LOG_DEBUG,
                   "Video frame changed from size:%dx%d format:%s serial:%d to size:%dx%d format:%s serial:%d\n",
                   last_w, last_h,
                   (const char *)av_x_if_null(av_get_pix_fmt_name(last_format), "none"), last_serial,
                   frame->width, frame->height,
                (const char*)av_x_if_null(av_get_pix_fmt_name(frame->format), "none"), is->viddec.pkt_serial);

            //* 重新配置视频滤镜
            avfilter_graph_free(&graph); // 释放旧的滤镜图。
            graph = avfilter_graph_alloc(); // 分配一个新的滤镜图。 如果分配失败，返回 AVERROR(ENOMEM)。
            if (!graph) {
                ret = AVERROR(ENOMEM);
                goto the_end;
            }
            // 配置视频滤镜。
            graph->nb_threads = filter_nbthreads; // 滤镜图线程数
            if ((ret = configure_video_filters(graph, is, vfilters_list ? vfilters_list[is->vfilter_idx] : NULL, frame)) < 0) { /// 如果滤镜列表存在，则使用当前滤镜索引对应的滤镜；否则使用 NULL。
                // 配置失败，发送退出事件
                SDL_Event event;
                event.type = FF_QUIT_EVENT;
                event.user.data1 = is;
                SDL_PushEvent(&event);
                goto the_end;
            }
            // 记录滤镜图输入输出端
            filt_in = is->in_video_filter;
            filt_out = is->out_video_filter;
            // 更新缓存属性
            last_w = frame->width;
            last_h = frame->height;
            last_format = frame->format;
            last_serial = is->viddec.pkt_serial;
            last_vfilter_idx = is->vfilter_idx;
            // 从滤镜图的输出端获取帧率。
            frame_rate = av_buffersink_get_frame_rate(filt_out);
        }

        //*  将视频帧添加到滤镜图的输入缓冲区。
        ret = av_buffersrc_add_frame(filt_in, frame);
        if (ret < 0)
            goto the_end;

        while (ret >= 0) {
            FrameData *fd;

            is->frame_last_returned_time = av_gettime_relative() / 1000000.0;

            // 从滤镜图的输出缓冲区获取处理后的视频帧。
            ret = av_buffersink_get_frame_flags(filt_out, frame, 0);
            if (ret < 0) { // 
                if (ret == AVERROR_EOF)
                    is->viddec.finished = is->viddec.pkt_serial;
                ret = 0;
                break;
            }

            fd = frame->opaque_ref ? (FrameData*)frame->opaque_ref->data : NULL;

            // 计算帧的显示时间戳（PTS）和时长：
            is->frame_last_filter_delay = av_gettime_relative() / 1000000.0 - is->frame_last_returned_time;
            if (fabs(is->frame_last_filter_delay) > AV_NOSYNC_THRESHOLD / 10.0)
                is->frame_last_filter_delay = 0;
            tb = av_buffersink_get_time_base(filt_out);
            duration = (frame_rate.num && frame_rate.den ? av_q2d((AVRational){frame_rate.den, frame_rate.num}) : 0);
            pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
            // 将帧放入视频帧队列：
            ret = queue_picture(is, frame, pts, duration, fd ? fd->pkt_pos : -1, is->viddec.pkt_serial);
            // 释放帧引用：
            av_frame_unref(frame);
            if (is->videoq.serial != is->viddec.pkt_serial)
                break;
        }

        if (ret < 0)
            goto the_end;
    }
 the_end:
    avfilter_graph_free(&graph);
    av_frame_free(&frame);
    return 0;
}

static int subtitle_thread(void *arg)
{
    VideoState *is = arg;
    Frame *sp;
    int got_subtitle;
    double pts;

    for (;;) {
        if (!(sp = frame_queue_peek_writable(&is->subpq)))
            return 0;

        if ((got_subtitle = decoder_decode_frame(&is->subdec, NULL, &sp->sub)) < 0)
            break;

        pts = 0;

        if (got_subtitle && sp->sub.format == 0) {
            if (sp->sub.pts != AV_NOPTS_VALUE)
                pts = sp->sub.pts / (double)AV_TIME_BASE;
            sp->pts = pts;
            sp->serial = is->subdec.pkt_serial;
            sp->width = is->subdec.avctx->width;
            sp->height = is->subdec.avctx->height;
            sp->uploaded = 0;

            /* now we can update the picture count */
            frame_queue_push(&is->subpq);
        } else if (got_subtitle) {
            avsubtitle_free(&sp->sub);
        }
    }
    return 0;
}

/* copy samples for viewing in editor window */
//* 将音频样本数据复制到用于显示音频波形的缓冲区中。
static void update_sample_display(VideoState* is, short* samples, int samples_size)
{
    int size, len;

    //* 计算音频样本的数量。
    size = samples_size / sizeof(short);
    //* 将音频样本数据复制到环形缓冲区中。
    while (size > 0) {
        // 计算本次复制的长度
        len = SAMPLE_ARRAY_SIZE - is->sample_array_index;
        if (len > size)
            len = size;
        //  将 samples 中的数据复制到环形缓冲区 is->sample_array 中。
        memcpy(is->sample_array + is->sample_array_index, samples, len * sizeof(short));
        // 更新指针和索引
        samples += len;
        is->sample_array_index += len;
        // 处理环形缓冲区的回绕
        if (is->sample_array_index >= SAMPLE_ARRAY_SIZE)
            is->sample_array_index = 0;
        // 更新剩余样本数量：
        size -= len;
    }
}

/* return the wanted number of samples to get better sync if sync_type is video
  or external master clock */
  //* 根据主时钟（视频时钟或外部时钟）调整音频样本数量，以实现音视频同步。
// nb_samples：当前音频帧的样本数量。
static int synchronize_audio(VideoState* is, int nb_samples)
{
    //* 初始化 wanted_nb_samples 为当前音频帧的样本数量。
    int wanted_nb_samples = nb_samples;

    /* if not master, then we try to remove or add samples to correct the clock */
    //* 如果当前的主时钟不是音频时钟（AV_SYNC_AUDIO_MASTER），则需要同步音频 尝试删除或添加样本以校正时钟。
    if (get_master_sync_type(is) != AV_SYNC_AUDIO_MASTER) {
        double diff, avg_diff;
        int min_nb_samples, max_nb_samples;

        //* 计算音频时钟与主时钟的差值。    
        diff = get_clock(&is->audclk) - get_master_clock(is);

        
        if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD) { // 检查时钟差值是否有效且小于阈值。
            is->audio_diff_cum = diff + is->audio_diff_avg_coef * is->audio_diff_cum; // 更新差值的累积值。
            //* 检查是否有足够的样本用于估计差值。
            if (is->audio_diff_avg_count < AUDIO_DIFF_AVG_NB) {// 样本不足，增加计数。
                /* not enough measures to have a correct estimate */
                is->audio_diff_avg_count++; 
            } else { // 估计差值并调整音频样本数量。
                /* estimate the A-V difference */
                avg_diff = is->audio_diff_cum * (1.0 - is->audio_diff_avg_coef); // 差值的估计值。

                if (fabs(avg_diff) >= is->audio_diff_threshold) { // 差值超过阈值，需要调整样本数量。
                    wanted_nb_samples = nb_samples + (int)(diff * is->audio_src.freq); // 调整后的样本数量。
                    //min_nb_samples 和 max_nb_samples 样本数量的最小值和最大值，用于限制调整范围。
                    min_nb_samples = ((nb_samples * (100 - SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    max_nb_samples = ((nb_samples * (100 + SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    wanted_nb_samples = av_clip(wanted_nb_samples, min_nb_samples, max_nb_samples); // av_clip：将 wanted_nb_samples 限制在 min_nb_samples 和 max_nb_samples 之间。
                }
                av_log(NULL, AV_LOG_TRACE, "diff=%f adiff=%f sample_diff=%d apts=%0.3f %f\n",
                        diff, avg_diff, wanted_nb_samples - nb_samples,
                        is->audio_clock, is->audio_diff_threshold);
            }
        } else { // 处理差值过大情况, 可能是初始 PTS 错误，因此重置差值累积值和计数。
            /* too big difference : may be initial PTS errors, so
               reset A-V filter */
            is->audio_diff_avg_count = 0;
            is->audio_diff_cum       = 0;
        }
    }

    return wanted_nb_samples;
}

/**
 * Decode one audio frame and return its uncompressed size.
 *      //* 解码一个音频帧并返回其未压缩的大小。
 *
 * The processed audio frame is decoded, converted if required, and
 * stored in is->audio_buf, with size in bytes given by the return
 * value.
 *  //* 处理后的音频帧将被解码、转换（如果需要），并存储在is->audio_buf中，其大小由返回值指定。
 */
static int
audio_decode_frame(VideoState* is)
{
    int data_size, resampled_data_size; // data_size：原始音频帧的大小。  resampled_data_size：重采样后的音频数据大小。
    av_unused double audio_clock0; // 用于调试的音频时钟（未使用）。
    int wanted_nb_samples; // 期望的音频样本数（用于同步）。
    Frame *af; // 指向当前音频帧的指针。

    //* 如果播放器处于暂停状态，直接返回 -1。
    if (is->paused)
        return -1;

    //* 从音频帧队列中获取一个可读帧。
    do {
#if defined(_WIN32) // 在 Windows 平台上，如果队列为空，等待一段时间（av_usleep(1000)），如果超时则返回 -1。
        while (frame_queue_nb_remaining(&is->sampq) == 0) {
            if ((av_gettime_relative() - audio_callback_time) > 1000000LL * is->audio_hw_buf_size / is->audio_tgt.bytes_per_sec / 2)
                return -1;
            av_usleep (1000);
        }
#endif
        if (!(af = frame_queue_peek_readable(&is->sampq))) // 获取可读帧。
            return -1; // 获取失败，返回 -1。
        frame_queue_next(&is->sampq); // 移动到下一帧。
    } while (af->serial != is->audioq.serial); //解码帧不匹配packet队列序列号时则不继续处理，持续获取帧，直到帧匹配或packet队列停止

    //* 计算原始音频帧的大小。
    data_size = av_samples_get_buffer_size(NULL, af->frame->ch_layout.nb_channels,
                                           af->frame->nb_samples,
                                           af->frame->format, 1);

    //* 根据音频时钟同步，计算期望的样本数。
    wanted_nb_samples = synchronize_audio(is, af->frame->nb_samples);

    //* 检查是否需要重新初始化音频重采样器（swr_ctx）
    if ( // 音频帧的格式、通道布局或采样率与目标格式不一致。
        af->frame->format != is->audio_src.fmt ||
        av_channel_layout_compare(&af->frame->ch_layout, &is->audio_src.ch_layout) ||
        af->frame->sample_rate != is->audio_src.freq ||
        // 期望的样本数与实际样本数不一致
        // 且未初始化重采样器。
        (wanted_nb_samples != af->frame->nb_samples && !is->swr_ctx)) {
        //* 初始化音频重采样器。
        int ret;
        // 释放旧的重采样器（swr_free）。
        swr_free(&is->swr_ctx);
        // 使用 swr_alloc_set_opts2 创建新的重采样器。
        ret = swr_alloc_set_opts2(&is->swr_ctx,
                            &is->audio_tgt.ch_layout, is->audio_tgt.fmt, is->audio_tgt.freq,
                            &af->frame->ch_layout, af->frame->format, af->frame->sample_rate,
                            0, NULL);
        if (ret < 0 || swr_init(is->swr_ctx) < 0) { // 如果初始化失败，记录错误日志并返回 -1。
            av_log(NULL, AV_LOG_ERROR,
                   "Cannot create sample rate converter for conversion of %d Hz %s %d channels to %d Hz %s %d channels!\n",
                    af->frame->sample_rate, av_get_sample_fmt_name(af->frame->format), af->frame->ch_layout.nb_channels,
                    is->audio_tgt.freq, av_get_sample_fmt_name(is->audio_tgt.fmt), is->audio_tgt.ch_layout.nb_channels);
            swr_free(&is->swr_ctx);
            return -1;
        }
        // 更新音频源格式信息（is->audio_src）。
        if (av_channel_layout_copy(&is->audio_src.ch_layout, &af->frame->ch_layout) < 0)
            return -1;
        is->audio_src.freq = af->frame->sample_rate;
        is->audio_src.fmt = af->frame->format;
    }

    //* 如果需要重采样，则进行音频重采样；否则直接使用原始音频数据。
    if (is->swr_ctx) {
        const uint8_t **in = (const uint8_t **)af->frame->extended_data;
        // 设置输出缓冲区
        uint8_t **out = &is->audio_buf1;
        int out_count = (int64_t)wanted_nb_samples * is->audio_tgt.freq / af->frame->sample_rate + 256;
        int out_size  = av_samples_get_buffer_size(NULL, is->audio_tgt.ch_layout.nb_channels, out_count, is->audio_tgt.fmt, 0);
        int len2;
        if (out_size < 0) {
            av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size() failed\n");
            return -1;
        }
        // 如果期望样本数与实际样本数不一致，设置重采样补偿（swr_set_compensation）。
        if (wanted_nb_samples != af->frame->nb_samples) {
            if (swr_set_compensation(is->swr_ctx, (wanted_nb_samples - af->frame->nb_samples) * is->audio_tgt.freq / af->frame->sample_rate,
                                        wanted_nb_samples * is->audio_tgt.freq / af->frame->sample_rate) < 0) {
                av_log(NULL, AV_LOG_ERROR, "swr_set_compensation() failed\n");
                return -1;
            }
        }
        // 分配输出缓冲区。
        av_fast_malloc(&is->audio_buf1, &is->audio_buf1_size, out_size);
        if (!is->audio_buf1)
            return AVERROR(ENOMEM);
        // 进行音频重采样。
        len2 = swr_convert(is->swr_ctx, out, out_count, in, af->frame->nb_samples);
        if (len2 < 0) {
            av_log(NULL, AV_LOG_ERROR, "swr_convert() failed\n");
            return -1;
        }
        if (len2 == out_count) {
            av_log(NULL, AV_LOG_WARNING, "audio buffer is probably too small\n");
            if (swr_init(is->swr_ctx) < 0)
                swr_free(&is->swr_ctx);
        }
        // 更新 is->audio_buf 和 resampled_data_size。
        is->audio_buf = is->audio_buf1;
        resampled_data_size = len2 * is->audio_tgt.ch_layout.nb_channels * av_get_bytes_per_sample(is->audio_tgt.fmt);
    } else {
        is->audio_buf = af->frame->data[0];
        resampled_data_size = data_size;
    }

    //* 更新音频时钟。
    audio_clock0 = is->audio_clock;
    /* update the audio clock with the pts */
    if (!isnan(af->pts)) // 如果音频帧的 PTS 有效（!isnan(af->pts)）
        is->audio_clock = af->pts + (double) af->frame->nb_samples / af->frame->sample_rate; // 更新音频时钟（is->audio_clock）。
    else
        is->audio_clock = NAN; // 将音频时钟设置为 NAN。
    // 更新音频时钟的序列号（is->audio_clock_serial）。
    is->audio_clock_serial = af->serial;
    //* 在调试模式下打印音频时钟信息。
#ifdef DEBUG
    {
        static double last_clock;
        printf("audio: delay=%0.3f clock=%0.3f clock0=%0.3f\n",
               is->audio_clock - last_clock,
               is->audio_clock, audio_clock0);
        last_clock = is->audio_clock;
    }
#endif
    return resampled_data_size;
}

/* prepare a new audio buffer */
//* 在音频设备需要数据时被调用，填充音频缓冲区（stream）。
// 从音频队列中解码音频数据。
// 将解码后的音频数据填充到 stream 中。
// 处理静音、音量调整等音频效果。
// 更新音频时钟（audio_clock）以实现音视频同步。
static void sdl_audio_callback(void* opaque, Uint8* stream, int len)
{
    VideoState *is = opaque;
    int audio_size, len1; // audio_size：用于存储解码后的音频数据大小。 len1：用于存储本次需要填充的音频数据长度。

    //* 记录回调时间, 为了音视频同步时更加精确
    audio_callback_time = av_gettime_relative();

    //* 填充音频缓冲区
    while (len > 0) { //* 循环填充音频缓冲区，直到 stream 被填满（len 为 0）。
        //* 处理用户音频缓冲区中的数据被读完的情况
        if (is->audio_buf_index >= is->audio_buf_size) { // 表示缓冲区已耗尽，需要解码新的音频数据。
            // 从帧队列中获取帧并解码到音频缓冲区中
           audio_size = audio_decode_frame(is); 
           if (audio_size < 0) {
                /* if error, just output silence */
               is->audio_buf = NULL; // 将 is->audio_buf 设置为 NULL，表示没有有效音频数据。
               is->audio_buf_size = SDL_AUDIO_MIN_BUFFER_SIZE / is->audio_tgt.frame_size * is->audio_tgt.frame_size; // 设置 is->audio_buf_size 为默认的缓冲区大小（SDL_AUDIO_MIN_BUFFER_SIZE 的整数倍）。
           } else { // 如果解码成功，更新音频缓冲区大小并显示音频波形（如果需要）
               if (is->show_mode != SHOW_MODE_VIDEO)
                   update_sample_display(is, (int16_t *)is->audio_buf, audio_size);
               is->audio_buf_size = audio_size;
           }
           // 重置音频缓冲区的读取位置（is->audio_buf_index）为 0。
           is->audio_buf_index = 0; 
        }
        //* 计算本次需要填充的音频数据长度
        // len1 为用户音频缓冲区的剩余数据长度。
        len1 = is->audio_buf_size - is->audio_buf_index;
        if (len1 > len) // len为SDL回调音频的缓冲区大小，一次填充不能超过这个大小
            len1 = len;
        //* 将音频数据填充到 stream 中
        if (!is->muted && is->audio_buf && is->audio_volume == SDL_MIX_MAXVOLUME) // 未静音、音频缓冲区有效且音量为最大值
            memcpy(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len1); // 直接使用 memcpy 复制数据。
        else { // 否则，先将 stream 清零（memset），然后使用 SDL_MixAudioFormat 混合音频数据（支持音量调整）。
            memset(stream, 0, len1);
            if (!is->muted && is->audio_buf) //未静音、音频缓冲区有效
                SDL_MixAudioFormat(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, AUDIO_S16SYS, len1, is->audio_volume);
        }
        //* 更新 stream 指针、剩余长度（len）和音频缓冲区索引（is->audio_buf_index）。
        len -= len1;
        stream += len1;
        is->audio_buf_index += len1;
    }
    //* 更新剩余未写入的音频数据大小
    is->audio_write_buf_size = is->audio_buf_size - is->audio_buf_index;
    //* 更新音频时钟并同步外部时钟。
    /* Let's assume the audio driver that is used by SDL has two periods. */ //* 假设SDL使用的音频驱动程序有两个周期
    if (!isnan(is->audio_clock)) { // audio_clock 有效
        // set_clock_at 更新音频时钟（is->audclk）。
        set_clock_at(&is->audclk, is->audio_clock - (double)(2 * is->audio_hw_buf_size + is->audio_write_buf_size) / is->audio_tgt.bytes_per_sec, is->audio_clock_serial, audio_callback_time / 1000000.0);
        // sync_clock_to_slave 将外部时钟（is->extclk）同步到音频时钟。
        sync_clock_to_slave(&is->extclk, &is->audclk);
    }
}

//* 用于打开音频设备的函数, 参数来自过滤器链的输出端
// void *opaque: 一个不透明的指针，通常用于传递用户自定义数据。
// AVChannelLayout *wanted_channel_layout: 指向 AVChannelLayout 结构的指针，表示期望的音频通道布局。
// int wanted_sample_rate: 期望的采样率。
// struct AudioParams *audio_hw_params: 指向 AudioParams 结构的指针，用于存储最终的音频硬件参数。
static int audio_open(void* opaque, AVChannelLayout* wanted_channel_layout, int wanted_sample_rate, struct AudioParams* audio_hw_params)
{
    SDL_AudioSpec wanted_spec, spec; // wanted_spec 用于存储期望的音频参数，spec 用于存储实际获得的音频参数。
    const char *env; // 存储环境变量的值。
    static const int next_nb_channels[] = {0, 0, 1, 6, 2, 6, 4, 6}; // 无法打开音频设备时尝试的下一个通道数。
    static const int next_sample_rates[] = {0, 44100, 48000, 96000, 192000}; // 在无法打开音频设备时尝试的下一个采样率。
    int next_sample_rate_idx = FF_ARRAY_ELEMS(next_sample_rates) - 1; // 下一个尝试的采样率在数组中的索引，初始值为数组的最后一个元素的索引。
    int wanted_nb_channels = wanted_channel_layout->nb_channels; // 期望的通道数，初始值为 wanted_channel_layout 中的通道数。

    //* 打开音频设备的参数设置
    // 音频通道布局检查
    env = SDL_getenv("SDL_AUDIO_CHANNELS"); // 通过环境变量 SDL_AUDIO_CHANNELS，用户可以动态调整音频通道数，而不需要修改代码或重新编译程序。
    if (env) { // 用用户指定的通道数来布局音频通道
        wanted_nb_channels = atoi(env); 
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, wanted_nb_channels);
    } // 如果用户指定的通道数的布局顺序不是原生布局，可能不被硬件所支持，后续打开音频设备失败，则用原来的通道数重新布局音频通道
    if (wanted_channel_layout->order != AV_CHANNEL_ORDER_NATIVE) {
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, wanted_nb_channels);
    }
    wanted_nb_channels = wanted_channel_layout->nb_channels;
    wanted_spec.channels = wanted_nb_channels;
    wanted_spec.freq = wanted_sample_rate;
    // 检查最终设置的采样率和通道数是否可用
    if (wanted_spec.freq <= 0 || wanted_spec.channels <= 0) {
        av_log(NULL, AV_LOG_ERROR, "Invalid sample rate or channel count!\n");
        return -1;
    }
    // 在采样率档位列表中找到符合所需采样率的档位
    while (next_sample_rate_idx && next_sample_rates[next_sample_rate_idx] >= wanted_spec.freq)
        next_sample_rate_idx--;
    wanted_spec.format = AUDIO_S16SYS; // 16位系统字节序
    wanted_spec.silence = 0;    // 静音值为0。
    // 设置音频缓冲区的大小，取 SDL_AUDIO_MIN_BUFFER_SIZE 和 2 << av_log2(wanted_spec.freq / SDL_AUDIO_MAX_CALLBACKS_PER_SEC) 中的较大值。
    wanted_spec.samples = FFMAX(SDL_AUDIO_MIN_BUFFER_SIZE, 2 << av_log2(wanted_spec.freq / SDL_AUDIO_MAX_CALLBACKS_PER_SEC));
    wanted_spec.callback = sdl_audio_callback; // 音频回调函数
    wanted_spec.userdata = opaque; // 用户数据

    
    //* 使用设置的参数打开音频设备
    // 打开失败时允许SDL改变采样率和通道数重新尝试打开
    while (!(audio_dev = SDL_OpenAudioDevice(NULL, 0, &wanted_spec, &spec, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE | SDL_AUDIO_ALLOW_CHANNELS_CHANGE))) {
        av_log(NULL, AV_LOG_WARNING, "SDL_OpenAudio (%d channels, %d Hz): %s\n",
               wanted_spec.channels, wanted_spec.freq, SDL_GetError()); // 输出警告日志，显示当前尝试的通道数和采样率，以及错误信息。
        wanted_spec.channels = next_nb_channels[FFMIN(7, wanted_spec.channels)]; // 尝试下一个通道数。
        if (!wanted_spec.channels) { // 如果没有更多的通道数可尝试，则尝试下一个采样率
            wanted_spec.freq = next_sample_rates[next_sample_rate_idx--]; // 设置下一个采样率
            wanted_spec.channels = wanted_nb_channels; // 重置通道数为期望的通道数。
            if (!wanted_spec.freq) { // 如果没有更多的采样率可尝试，则输出错误日志并返回-1。
                av_log(NULL, AV_LOG_ERROR,
                       "No more combinations to try, audio open failed\n");
                return -1;
            }
        }
        // 重新初始化通道布局
        av_channel_layout_default(wanted_channel_layout, wanted_spec.channels);
    }

    //* 打开音频设备成功, 检查实际获得的音频参数是否被SDL支持
    if (spec.format != AUDIO_S16SYS) { // 音频格式
        av_log(NULL, AV_LOG_ERROR,
               "SDL advised audio format %d is not supported!\n", spec.format);
        return -1;
    }
    if (spec.channels != wanted_spec.channels) { //音频通道数
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, spec.channels); // 实际获得的通道数重新初始化 wanted_channel_layout
        if (wanted_channel_layout->order != AV_CHANNEL_ORDER_NATIVE) { // 通道布局的顺序不是原生的
            av_log(NULL, AV_LOG_ERROR,
                   "SDL advised channel count %d is not supported!\n", spec.channels);
            return -1;
        }
    }

    //* 从实际打开音频设备的参数，设置音频硬件参数
    audio_hw_params->fmt = AV_SAMPLE_FMT_S16;
    audio_hw_params->freq = spec.freq;
    if (av_channel_layout_copy(&audio_hw_params->ch_layout, wanted_channel_layout) < 0)
        return -1;
    audio_hw_params->frame_size = av_samples_get_buffer_size(NULL, audio_hw_params->ch_layout.nb_channels, 1, audio_hw_params->fmt, 1);
    audio_hw_params->bytes_per_sec = av_samples_get_buffer_size(NULL, audio_hw_params->ch_layout.nb_channels, audio_hw_params->freq, audio_hw_params->fmt, 1);
    if (audio_hw_params->bytes_per_sec <= 0 || audio_hw_params->frame_size <= 0) {
        av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size failed\n");
        return -1;
    }
    return spec.size;
}

//* 创建硬件加速设备上下文（Hardware Device Context），用于支持视频解码的硬件加速。
// AVBufferRef **device_ctx：指向硬件设备上下文的指针。
static int create_hwaccel(AVBufferRef** device_ctx)
{
    enum AVHWDeviceType type;
    int ret;
    AVBufferRef *vk_dev;    //* 指向 Vulkan 设备上下文的指针。

    *device_ctx = NULL;

    //* 指向 Vulkan 设备上下文的指针。
    if (!hwaccel) //* 表示用户指定的硬件加速类型（如 "cuda"、"vulkan" 等）。
        return 0;

    //* 查找硬件设备类型
    type = av_hwdevice_find_type_by_name(hwaccel); // 根据 hwaccel 的名称查找对应的硬件设备类型。
    if (type == AV_HWDEVICE_TYPE_NONE) // 例如，"cuda" 对应 AV_HWDEVICE_TYPE_CUDA，"vulkan" 对应 AV_HWDEVICE_TYPE_VULKAN。
        return AVERROR(ENOTSUP);

    //*  获取 Vulkan 设备上下文
    ret = vk_renderer_get_hw_dev(vk_renderer, &vk_dev);
    if (ret < 0)
        return ret;

    //* 派生硬件设备上下文
    ret = av_hwdevice_ctx_create_derived(device_ctx, type, vk_dev, 0); // 尝试从 Vulkan 设备上下文派生出指定类型的硬件设备上下文。
    if (!ret)
        return 0;

    //*  处理派生失败的情况    
    if (ret != AVERROR(ENOSYS))
        return ret;
    av_log(NULL, AV_LOG_WARNING, "Derive %s from vulkan not supported.\n", hwaccel);

    //* 如果无法从 Vulkan 设备派生硬件设备上下文，则尝试直接创建。
    ret = av_hwdevice_ctx_create(device_ctx, type, NULL, NULL, 0);
    return ret;
}

/* open a given stream. Return 0 if OK */
//* 打开和初始化媒体流以便进行解码和播放
static int stream_component_open(VideoState* is, int stream_index)
{
    AVFormatContext *ic = is->ic;   // 媒体文件的格式上下文
    AVCodecContext *avctx;          // 编解码器上下文
    const AVCodec *codec;           // 编解码器
    const char *forced_codec_name = NULL;   // 强制使用的编解码器名称
    AVDictionary *opts = NULL;      // 编解码器选项字典
    const AVDictionaryEntry *t = NULL;  // 遍历选项字典的条目
    int sample_rate;                // 音频采样率。
    AVChannelLayout ch_layout = { 0 };  // 音频通道布局。
    int ret = 0;
    int stream_lowres = lowres; // 低分辨率模式设置。

    //* 检查 stream_index 是否有效
    if (stream_index < 0 || stream_index >= ic->nb_streams) 
        return -1;

    //* 分配编解码器上下文
    avctx = avcodec_alloc_context3(NULL);
    if (!avctx)
        return AVERROR(ENOMEM);

    //* 设置编解码器参数
    ret = avcodec_parameters_to_context(avctx, ic->streams[stream_index]->codecpar);
    if (ret < 0)
        goto fail;
    avctx->pkt_timebase = ic->streams[stream_index]->time_base; // 设置 pkt_timebase（时间基），用于时间戳的计算

    //* 查找解码器
    codec = avcodec_find_decoder(avctx->codec_id);

    //* 处理强制编解码器
    switch (avctx->codec_type) { // 根据流类型，更新 is->last_audio_stream、is->last_subtitle_stream 或 is->last_video_stream，记录当前流的索引。
                                // 设置 forced_codec_name 为对应的编解码器名称（audio_codec_name、video_codec_name 或 subtitle_codec_name）。
        case AVMEDIA_TYPE_AUDIO: is->last_audio_stream = stream_index; forced_codec_name = audio_codec_name; break;
        case AVMEDIA_TYPE_SUBTITLE: is->last_subtitle_stream = stream_index; forced_codec_name = subtitle_codec_name; break;
        case AVMEDIA_TYPE_VIDEO   : is->last_video_stream    = stream_index; forced_codec_name =    video_codec_name; break;
    }
    if (forced_codec_name) // 如果指定了强制编解码器名称，尝试按名称查找解码器。
        codec = avcodec_find_decoder_by_name(forced_codec_name);
    if (!codec) {   // 如果找不到解码器，记录警告并返回错误。
        if (forced_codec_name) av_log(NULL, AV_LOG_WARNING,
                                      "No codec could be found with name '%s'\n", forced_codec_name);
        else                   av_log(NULL, AV_LOG_WARNING,
                                      "No decoder could be found for codec %s\n", avcodec_get_name(avctx->codec_id));
        ret = AVERROR(EINVAL);
        goto fail;
    }

    //* 设置编解码器的 ID 和低分辨率模式
    avctx->codec_id = codec->id;    // 将找到的解码器的 ID 赋值给 avctx->codec_id。
    if (stream_lowres > codec->max_lowres) { // 检查请求的低分辨率是否超出解码器支持的最大值，如果超出则调整到支持的最大值。
        av_log(avctx, AV_LOG_WARNING, "The maximum value for lowres supported by the decoder is %d\n",
                codec->max_lowres);
        stream_lowres = codec->max_lowres;
    }
    avctx->lowres = stream_lowres;  // 最终的低分辨率值设置到 avctx->lowres 中

    //* 优化解码性能的一种方式，适用于对解码速度要求较高的场景。
    if (fast)
        avctx->flags2 |= AV_CODEC_FLAG2_FAST;

    //* 过滤并设置编解码器选项
    ret = filter_codec_opts(codec_opts, avctx->codec_id, ic,
                            ic->streams[stream_index], codec, &opts);
    if (ret < 0)
        goto fail;

    //* 设置编解码器选项
    //  检查并设置线程数
    if (!av_dict_get(opts, "threads", NULL, 0)) // 如果 opts 中没有设置 threads 选项，则将其设置为 "auto"，表示自动选择线程数。
        av_dict_set(&opts, "threads", "auto", 0);
    // 设置低分辨率模式
    if (stream_lowres)
        av_dict_set_int(&opts, "lowres", stream_lowres, 0);
    // 编解码器在处理数据时，复制不透明的数据（opaque data）。
    // 不透明的数据通常是编解码器内部使用的私有数据，不会直接暴露给用户。
    // 启用 copy_opaque 标志后，编解码器会将这些数据从输入帧复制到输出帧。
    av_dict_set(&opts, "flags", "+copy_opaque", AV_DICT_MULTIKEY);

    //* 为视频流创建编解码硬件加速上下文
    if (avctx->codec_type == AVMEDIA_TYPE_VIDEO) {
        ret = create_hwaccel(&avctx->hw_device_ctx);
        if (ret < 0)
            goto fail;
    }

    //* 打开编解码器并检查编解码器选项是否被正确使用。
    if ((ret = avcodec_open2(avctx, codec, &opts)) < 0) {
        goto fail;
    }// 编解码器选项中存在未被使用的选项，记录错误并返回。
    if ((t = av_dict_get(opts, "", NULL, AV_DICT_IGNORE_SUFFIX))) { // 空字符串表示匹配所有键。AV_DICT_IGNORE_SUFFIX：忽略键的后缀。
        av_log(NULL, AV_LOG_ERROR, "Option %s not found.\n", t->key); // 如果编解码器选项中存在未被使用的选项，可能是用户输入错误或不支持的选项。
        ret =  AVERROR_OPTION_NOT_FOUND;
        goto fail;
    }

    //* 表示当前流尚未到达文件末尾（EOF）。
    is->eof = 0; // 打开一个新的流组件时，需要确保 eof 标志被重置为 0，以便后续的读取和处理逻辑能够正常工作。
    //* 默认的丢弃策略
    ic->streams[stream_index]->discard = AVDISCARD_DEFAULT;

    switch (avctx->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
    //* 配置音频过滤器
        {
            AVFilterContext *sink;

            //* 设置音频过滤器源参数
            is->audio_filter_src.freq = avctx->sample_rate;
            ret = av_channel_layout_copy(&is->audio_filter_src.ch_layout, &avctx->ch_layout); // 从编解码器中拷贝通道布局到输入过滤器中
            if (ret < 0)
                goto fail;
            is->audio_filter_src.fmt            = avctx->sample_fmt;
            //*  配置音频过滤器链
            if ((ret = configure_audio_filters(is, afilters, 0)) < 0) // 配置音频过滤器链。
                goto fail;
            sink = is->out_audio_filter;    // 获取过滤器链的输出端。
            sample_rate    = av_buffersink_get_sample_rate(sink);   // 从过滤器链输出端获取采样率。
            ret = av_buffersink_get_ch_layout(sink, &ch_layout);    // 从过滤器链输出端获取声道布局。
            if (ret < 0)
                goto fail;
        }

        /* prepare audio output */
        //*  初始化音频输出
        // 以过滤器链输出端的音频参数打开音频设备
        if ((ret = audio_open(is, &ch_layout, sample_rate, &is->audio_tgt)) < 0) 
            goto fail;
        is->audio_hw_buf_size = ret;    // 存储音频硬件缓冲区的大小。
        is->audio_src = is->audio_tgt;  // 设置音频源参数为目标参数。
        is->audio_buf_size  = 0;        // 初始化音频缓冲区的状态。
        is->audio_buf_index = 0;

        /* init averaging filter */
        //* 初始化音频平滑参数
        is->audio_diff_avg_coef = exp(log(0.01) / AUDIO_DIFF_AVG_NB); // 计算音频平滑滤波系数
        is->audio_diff_avg_count = 0;                                   // 初始化平滑滤波计数器。
        /* since we do not have a precise anough audio FIFO fullness,
           we correct audio sync only if larger than this threshold */
        is->audio_diff_threshold = (double)(is->audio_hw_buf_size) / is->audio_tgt.bytes_per_sec; // 设置音频平滑阈值，用于判断是否需要调整平滑。

        //* 初始化音频解码器
        // 设置音频流的索引和流对象。
        is->audio_stream = stream_index;
        is->audio_st = ic->streams[stream_index];

        if ((ret = decoder_init(&is->auddec, avctx, &is->audioq, is->continue_read_thread)) < 0) // 初始化音频解码器。
            goto fail;
        if (is->ic->iformat->flags & AVFMT_NOTIMESTAMPS) { // 如果输入格式没有时间戳，手动设置解码器的起始时间戳。
            is->auddec.start_pts = is->audio_st->start_time;
            is->auddec.start_pts_tb = is->audio_st->time_base;
        }
        // 启动音频解码线程（audio_thread）。
        if ((ret = decoder_start(&is->auddec, audio_thread, "audio_decoder", is)) < 0)
            goto out;
        // 启动音频设备，开始播放音频。
        SDL_PauseAudioDevice(audio_dev, 0);
        break;
    case AVMEDIA_TYPE_VIDEO:
        // 设置视频流的索引和流对象。
        is->video_stream = stream_index;
        is->video_st = ic->streams[stream_index];

        // 初始化视频解码器。
        if ((ret = decoder_init(&is->viddec, avctx, &is->videoq, is->continue_read_thread)) < 0)
            goto fail;
        // 启动视频解码线程（video_thread）。
        if ((ret = decoder_start(&is->viddec, video_thread, "video_decoder", is)) < 0)
            goto out;
        // 标记需要处理视频流中的附件（如封面图片）
        is->queue_attachments_req = 1;
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        // 设置字幕流的索引和流对象。
        is->subtitle_stream = stream_index;
        is->subtitle_st = ic->streams[stream_index];

        // 初始化字幕解码器。
        if ((ret = decoder_init(&is->subdec, avctx, &is->subtitleq, is->continue_read_thread)) < 0)
            goto fail;
        // 启动字幕解码线程（subtitle_thread）。
        if ((ret = decoder_start(&is->subdec, subtitle_thread, "subtitle_decoder", is)) < 0)
            goto out;
        break;
    default:
        break;
    }
    goto out;

fail:
    avcodec_free_context(&avctx);
out:
    av_channel_layout_uninit(&ch_layout);
    av_dict_free(&opts);

    return ret;
}

static int decode_interrupt_cb(void *ctx)
{
    VideoState *is = ctx;
    return is->abort_request;
}

//* 判断某个流的数据包是否足够的函数
// 如果数据包足够，则返回 1；否则返回 0。
static int stream_has_enough_packets(AVStream* st, int stream_id, PacketQueue* queue)
{
    return stream_id < 0 || // 该流不存在（例如，媒体文件中没有音频流或字幕流）, 直接返回 1，表示“数据足够”（因为没有数据需要读取）
           queue->abort_request || // 队列已中止。 在这种情况下，直接返回 1，表示“数据足够”（因为不需要继续读取数据）
           (st->disposition & AV_DISPOSITION_ATTACHED_PIC) || // 表示该流是附加图片（如封面图）。在这种情况下，直接返回 1，表示“数据足够”（因为附加图片只需要读取一次）。
        queue->nb_packets > MIN_FRAMES && (!queue->duration || av_q2d(st->time_base) * queue->duration > 1.0); // 检查数据包数量和时长
    //* queue->nb_packets > MIN_FRAMES  队列中的数据包数量是否超过最小帧数      
    //* !queue->duration 队列的总时长（queue->duration）为 0，表示时长未知。在这种情况下，只要数据包数量足够，就认为数据足够。
    //* av_q2d(st->time_base) * queue->duration > 1.0： 如果总时长超过 1.0 秒，表示数据足够。
}

//* 通过检查输入格式的名称以及流的 URL 来确定流的类型。
static int is_realtime(AVFormatContext *s)
{
    //* 检查输入格式名称, 如果流是 RTP、RTSP 或 SDP 格式
    if ( !strcmp(s->iformat->name, "rtp")
        || !strcmp(s->iformat->name, "rtsp")
        || !strcmp(s->iformat->name, "sdp")
    )
        return 1;

    //* 检查 URL 协议, 其 URL 以 rtp : 或 udp : 开头
    if ( s->pb && (!strncmp(s->url, "rtp:", 4)
        || !strncmp(s->url, "udp:", 4)
        )
    )
        return 1;
    //* 则视为实时流，返回 1。
    //* 否则，返回 0。
    return 0;
}

/* this thread gets the stream from the disk or the network */
//* 多线程的入口回调函数，负责从磁盘或网络读取视频流。
//* 它的主要任务是打开媒体文件，读取流信息，并将数据包放入相应的队列中供其他线程使用。
static int read_thread(void *arg)
{
    VideoState *is = arg;
    AVFormatContext *ic = NULL;     //* 解码上下文
    int err, i, ret;
    int st_index[AVMEDIA_TYPE_NB];  //* 存储不同媒体类型流的索引
    AVPacket *pkt = NULL;           //* 存储读取到的数据包
    int64_t stream_start_time;      //* 流开始的时间戳
    int pkt_in_play_range = 0;      //* 流是否在用户指定的时间范围内
    const AVDictionaryEntry *t;

    //* 是否扫描所有节目
    int scan_all_pmts_set = 0;
    //* 包timestamp
    int64_t pkt_ts;

    //* 创建一个互斥锁用于 wait read
    SDL_mutex *wait_mutex = SDL_CreateMutex();
    if ( !wait_mutex ) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    //* 初始化 st_index 数组为 -1，表示尚未找到任何流
    memset(st_index, -1, sizeof(st_index));
    //* is->eof 设置为 0，表示未到达文件结束。
    is->eof = 0;

    //* 分配一个 AVPacket 数据包。如果分配失败，记录错误并处理失败。
    pkt = av_packet_alloc();
    if (!pkt) {
        av_log(NULL, AV_LOG_FATAL, "Could not allocate packet.\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    //*  FFmpeg 中用于初始化媒体文件解码上下文的典型实现。
    //* 创建解码上下文.如果分配失败，记录错误并处理失败。
    ic = avformat_alloc_context();
    if ( !ic ) {
        av_log(NULL, AV_LOG_FATAL, "Could not allocate context.\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    //* 设置解码中断回调,用于在读取过程中可以中断操作。
    // 当 FFmpeg 执行阻塞操作时，会定期调用 callback 函数。
    // 如果 callback 返回 1，则阻塞操作会被中断，并返回错误码 AVERROR_EXIT。
    // 如果 callback 返回 0，则阻塞操作会继续执行。
    ic->interrupt_callback.callback = decode_interrupt_cb;
    ic->interrupt_callback.opaque = is;
    //* 设置格式选项，是否扫描所有节目
    if (!av_dict_get(format_opts, "scan_all_pmts", NULL, AV_DICT_MATCH_CASE)) {
        av_dict_set(&format_opts, "scan_all_pmts", "1", AV_DICT_DONT_OVERWRITE);
        scan_all_pmts_set = 1;
    }
    //* 打开指定的文件或网络地址，获取流信息
    err = avformat_open_input(&ic, is->filename, is->iformat, &format_opts);
    if ( err < 0 ) {
        //* 如果打开失败，调用 print_error 函数输出错误信息并退出。
        print_error(is->filename, err);
        ret = -1;
        goto fail;
    }
    //* 如果之前设置了 scan_all_pmts 选项，则清除该选项。
    if ( scan_all_pmts_set )
        av_dict_set(&format_opts, "scan_all_pmts", NULL, AV_DICT_MATCH_CASE);
    //* 检查是否有未识别的格式选项，如果有则记录错误并处理失败。
    if ((t = av_dict_get(format_opts, "", NULL, AV_DICT_IGNORE_SUFFIX))) {
        av_log(NULL, AV_LOG_ERROR, "Option %s not found.\n", t->key);
        ret = AVERROR_OPTION_NOT_FOUND;
        goto fail;
    }
    //* 将解码上下文指针保存到 VideoState 结构中。
    is->ic = ic;

    //* 如果需要生成时间戳，则设置相应的标志。
    if ( genpts )
        ic->flags |= AVFMT_FLAG_GENPTS;

    //* 获取媒体文件的流信息
    if ( find_stream_info ) {
        // 媒体流信息选项的指针数组，每个媒体流对应一个AVDictionary*
        AVDictionary **opts;
        int orig_nb_streams = ic->nb_streams; // 保存原始的流数量（ic->nb_streams）

        //* 为每个流设置查找选项
        err = setup_find_stream_info_opts(ic, codec_opts, &opts);
        if (err < 0) {
            av_log(NULL, AV_LOG_ERROR,
                   "Error setting up avformat_find_stream_info() options\n");
            ret = err;
            goto fail;
        }

        //* 获取媒体文件的流信息（如编解码器参数、帧率、时长等）,设置到ic中
        err = avformat_find_stream_info(ic, opts);
        //* 释放媒体流选项指针数组
        for ( i = 0; i < orig_nb_streams; i++ )
            av_dict_free(&opts[i]); 
        av_freep(&opts);

        //* 检查并记录流的状态，如果找不到编解码参数，记录警告并退出
        if ( err < 0 ) {
            av_log(NULL, AV_LOG_WARNING,
                   "%s: could not find codec parameters\n", is->filename);
            ret = -1;
            goto fail;
        }
    }

    //* 将 AVIOContext 的 eof_reached 标志重置为 0，以清除文件结束（EOF）状态。
    if ( ic->pb ) //* 表示文件未到达结束位置。
        ic->pb->eof_reached = 0; // FIXME hack, ffplay maybe should not use avio_feof() to test for the end
                                        //* 这是一个临时解决方案（hack），用于解决某些情况下 avio_feof() 的错误使用。ffplay 可能不应该依赖 avio_feof() 来判断文件是否结束。
    //* 根据媒体文件的格式特性动态设置 seek_by_bytes 的值。
    //* seek_by_bytes 是一个标志，用于指示是否按字节进行 seek 操作。
    if (seek_by_bytes < 0)  //* 为负值, 根据媒体文件的格式特性动态设置其值。
        seek_by_bytes = !(ic->iformat->flags & AVFMT_NO_BYTE_SEEK) && //* 如果媒体文件格式支持按字节 seek
                        !!(ic->iformat->flags & AVFMT_TS_DISCONT) && //* 媒体文件格式的时间戳不连续
                        strcmp("ogg", ic->iformat->name); //* 媒体文件格式不是 ogg(不支持字节寻址)
        //* 则将 seek_by_bytes 设置为 1，否则设置为 0。
        
    //* 初始化最大帧持续时间，对于时间戳不连续的格式（如 MPEG-TS），帧的持续时间可能较短且不规律。
    //*  设置为较小的值（如 10.0 秒），可以避免因时间戳不连续导致的解码问题。避免跳帧
    //* 对于时间戳连续的格式（如 MP4、MKV），帧的持续时间通常较长且规律。
    //* 将 max_frame_duration 设置为较大的值（如 3600.0 秒），可以提高解码效率。
    // 如果媒体文件格式的时间戳不连续，即当时间戳 discontinuity (断点) 存在时，最大帧持续时间设置为 10 毫秒
    // 否则为3600毫秒。
    is->max_frame_duration = (ic->iformat->flags & AVFMT_TS_DISCONT) ? 10.0 : 3600.0;

    //* 设置窗口标题, 检查外部设置标题或文件元数据标题、
    //* 否则设置为 “标题 - 文件名”
    if ( !window_title && (t = av_dict_get(ic->metadata, "title", NULL, 0)) )
        window_title = av_asprintf("%s - %s", t->value, input_filename);

    /* if seeking requested, we execute it */
    //* 根据指定的起始时间（start_time）执行 seek 操作，将媒体文件的播放位置定位到指定时间点。
    if ( start_time != AV_NOPTS_VALUE ) { //* 如果不是未设置
        int64_t timestamp;
        timestamp = start_time; //* 指定的起始时间（以 AV_TIME_BASE 为单位）。
        /* add the stream start time */ //* 计算目标时间戳
        if (ic->start_time != AV_NOPTS_VALUE) //* 文件的起始时间
            timestamp += ic->start_time; //* 将 start_time 加上文件的起始时间（ic->start_time），得到目标时间戳 timestamp。
        ret = avformat_seek_file(ic, -1, INT64_MIN, timestamp, INT64_MAX, 0);
        if (ret < 0) {
            av_log(NULL, AV_LOG_WARNING, "%s: could not seek to position %0.3f\n",
                    is->filename, (double)timestamp / AV_TIME_BASE);
        }
    }

    //* 检查是否为实时流
    is->realtime = is_realtime(ic);

    //* 显示状态
    if ( show_status )
        av_dump_format(ic, 0, is->filename, 0); //* 输出媒体信息到控制台

    //* 根据用户的需求选择特定的流（如视频流、音频流等），并默认丢弃其他流。
    // 根据用户指定的流描述符（wanted_stream_spec）选择媒体文件中的流，并将选中的流索引存储在 st_index 数组中。
    for (i = 0; i < ic->nb_streams; i++) { // 遍历所有流
        // 获取流类型
        AVStream* st = ic->streams[i];
        enum AVMediaType type = st->codecpar->codec_type;
        // 设置流的丢弃标志，用于控制是否处理该流
        st->discard = AVDISCARD_ALL;
        // 根据用户指定的流类型（如视频、音频、字幕）更新对应的 st_index。
        if (type >= 0 && wanted_stream_spec[type] && st_index[type] == -1)
            // wanted_stream_spec[type]：检查用户是否指定了这个流类型的流描述符，  如 "v:0" 表示第一个视频流
            // st_index[type] == -1：确保该类型的流尚未被分配索引。
            if (avformat_match_stream_specifier(ic, st, wanted_stream_spec[type]) > 0) //* 检查当前流 st 是否与用户指定的流规范（wanted_stream_spec[type]）匹配。
                st_index[type] = i;
    }
    
    //* 用于检查用户指定的流规范（wanted_stream_spec）是否匹配任何实际的流
    //* 确保用户指定的流可用性
    for ( i = 0; i < AVMEDIA_TYPE_NB; i++ ) {
        if ( wanted_stream_spec[i] && st_index[i] == -1 ) {//* 用户指定了，但没有匹配到任何实际的流
            //* 记录错误并将其标记为无效(INT_MAX)
            av_log(NULL, AV_LOG_ERROR, "Stream specifier %s does not match any %s stream\n",
                wanted_stream_spec[i], av_get_media_type_string(i));
            st_index[i] = INT_MAX;
        }
    }

    //* 根据用户的流偏好和可用流选择最佳音频、视频和字幕流。将它们的索引存储在 st_index 数组中
    if ( !video_disable )
        st_index[AVMEDIA_TYPE_VIDEO] =
            av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO, //* 流的类型
                                st_index[AVMEDIA_TYPE_VIDEO], -1, NULL, 0); //* 指定的视频流索引, 未指定相关流（如音频流）, 未指定解码器, 标志位（未使用）
    if (!audio_disable)
        st_index[AVMEDIA_TYPE_AUDIO] =
            av_find_best_stream(ic, AVMEDIA_TYPE_AUDIO,
                                st_index[AVMEDIA_TYPE_AUDIO],
                                st_index[AVMEDIA_TYPE_VIDEO], //* 相关流索引（视频流）。
                                NULL, 0);
    if (!video_disable && !subtitle_disable)
        st_index[AVMEDIA_TYPE_SUBTITLE] =
            av_find_best_stream(ic, AVMEDIA_TYPE_SUBTITLE,
                                st_index[AVMEDIA_TYPE_SUBTITLE],
                                (st_index[AVMEDIA_TYPE_AUDIO] >= 0 ? //* 相关流索引（优先使用音频流，如果音频流未选择则使用视频流）
                                 st_index[AVMEDIA_TYPE_AUDIO] :
                                 st_index[AVMEDIA_TYPE_VIDEO]),
                                NULL, 0);

    //* 根据视频流的属性（如宽度、高度和宽高比）设置默认的窗口大小，保存显示模式到 is->show_mode 中
    is->show_mode = show_mode;  // 用于控制视频的显示方式（如正常显示、全屏显示等）
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {    // 检查视频流是否存在
        AVStream *st = ic->streams[st_index[AVMEDIA_TYPE_VIDEO]]; // 视频流（AVStream）。
        AVCodecParameters *codecpar = st->codecpar; // 视频流的编解码参数（AVCodecParameters），包含宽度、高度等信息。
        AVRational sar = av_guess_sample_aspect_ratio(ic, st, NULL);    // 视频流的样本宽高比（SAR），通过 av_guess_sample_aspect_ratio 计算得到。
        if (codecpar->width)
            set_default_window_size(codecpar->width, codecpar->height, sar); // 根据视频流的宽度、高度和宽高比设置默认窗口大小。
    }

    /* open the streams */
    //* 打开流组件
    //* 根据前面找到的流索引，逐个打开音频、视频和字幕流，
    if ( st_index[AVMEDIA_TYPE_AUDIO] >= 0 ) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_AUDIO]);
    }
    ret = -1;
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
        ret = stream_component_open(is, st_index[AVMEDIA_TYPE_VIDEO]);
    }
    //* 如果没有其他显示模式，则默认设置为视频模式（SHOW_MODE_VIDEO）或者关系图模式（SHOW_MODE_RDFT）。
    if ( is->show_mode == SHOW_MODE_NONE )
        is->show_mode = ret >= 0 ? SHOW_MODE_VIDEO : SHOW_MODE_RDFT;

    if (st_index[AVMEDIA_TYPE_SUBTITLE] >= 0) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_SUBTITLE]);
    }

    //* 检查是否成功打开了音频或视频流。如果两者都未成功，则记录致命错误消息，并跳转到清理部分。
    if ( is->video_stream < 0 && is->audio_stream < 0 ) {
        av_log(NULL, AV_LOG_FATAL, "Failed to open file '%s' or configure filtergraph\n",
               is->filename);
        ret = -1;
        goto fail;
    }

    //* 如果设置了 infinite_buffer 为负值而且流是实时流，则将其设置为 1，以便处理实时流时使用无限缓冲。
    if ( infinite_buffer < 0 && is->realtime )
        infinite_buffer = 1;

    //* 读取数据包并处理各种状态
    for ( ;;) {
        //* 检查是否有中止请求，如果有则退出循环。
        if ( is->abort_request )
            break;
        //* 检查播放状态是否发生变化(通过对比当前的pause状态和上一次的pause状态), 处理暂停和恢复播放的请求。
        if ( is->paused != is->last_paused ) {
            is->last_paused = is->paused;
            //* 如果暂停，则调用 av_read_pause，否则调用 av_read_play 继续播放。
            if (is->paused)
                is->read_pause_return = av_read_pause(ic);
            else
                av_read_play(ic);
        }
//* RTSP 和 MMSH 协议的特殊处理
#if CONFIG_RTSP_DEMUXER || CONFIG_MMSH_PROTOCOL
        //* 如果正在暂停并且是 RTSP 或 MMSH 流，等待 10 毫秒以避免频繁读取数据包。
        if ( is->paused &&
                (!strcmp(ic->iformat->name, "rtsp") ||
                 (ic->pb && !strncmp(input_filename, "mmsh:", 5)))) {
            /* wait 10 ms to avoid trying to get another packet */
            /* XXX: horrible */
            SDL_Delay(10);
            continue;
        }
#endif
        //* 处理SEEK请求，更新播放状态和清空相应的包队列。
        // 寻址（Seeking）是指用户在播放器中拖动进度条或跳转到指定时间点时触发的操作。
        if ( is->seek_req ) {
            //* 计算目标时间戳并调用 avformat_seek_file 进行寻址。
            int64_t seek_target = is->seek_pos; // 用户指定的目标时间戳（通常是秒或微秒）。
            int64_t seek_min = is->seek_rel > 0 ? seek_target - is->seek_rel + 2 : INT64_MIN; // 寻址的最小时间戳。
            int64_t seek_max = is->seek_rel < 0 ? seek_target - is->seek_rel - 2 : INT64_MAX;
// FIXME the +-2 is due to rounding being not done in the correct direction in generation
//      of the seek_pos/seek_rel variables
            // +2 和 -2 是修正值，用于解决时间戳舍入误差问题（代码注释中提到这是一个临时的修复）。

            //* 寻址
            ret = avformat_seek_file(is->ic, -1, seek_min, seek_target, seek_max, is->seek_flags); // 在媒体文件中进行寻址。 -1：表示对所有流进行寻址（而不是针对某个特定的流）。
            if ( ret < 0 ) { // 寻址失败
                av_log(NULL, AV_LOG_ERROR,
                       "%s: error while seeking\n", is->ic->url);
            } else { // 寻址成功
                //* 如果寻址成功，清空音频、视频和字幕的包队列
                // 队列中的数据已经失效（它们属于寻址前的时间段）。
                if (is->audio_stream >= 0)
                    packet_queue_flush(&is->audioq);
                if ( is->subtitle_stream >= 0 )
                    packet_queue_flush(&is->subtitleq);
                if ( is->video_stream >= 0 )
                    packet_queue_flush(&is->videoq);
                //* 根据寻址标志更新时钟。
                if ( is->seek_flags & AVSEEK_FLAG_BYTE ) { // 如果是按字节寻址，则将外部时钟设置为无效值（NAN）。
                    set_clock(&is->extclk, NAN, 0);
                } else { // 否则，将外部时钟设置为目标时间戳（seek_target），并将其转换为秒（除以 AV_TIME_BASE）
                    set_clock(&is->extclk, seek_target / (double) AV_TIME_BASE, 0);
                }
            }

            is->seek_req = 0; // 寻址请求已处理完毕。
            is->queue_attachments_req = 1; // 需要重新加载附件（如封面图片）。
            is->eof = 0; // 重置 EOF（文件结束）标志。
            if ( is->paused ) // 如果播放器处于暂停状态，调用 step_to_next_frame 跳转到下一帧（以便立即显示寻址后的画面）。
                step_to_next_frame(is);
        }

        //* 检查媒体文件中是否存在附加图片（例如封面图），并将其放入视频队列中，以便后续解码和显示。
        // 附加图片通常存储在 AVStream 的 attached_pic 字段中 , 通过disposition字段用 AV_DISPOSITION_ATTACHED_PIC 标志标识。
        if (is->queue_attachments_req) {
            //* 检查是否有请求附加图片（如封面图）
            if ( is->video_st && is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC ) {
                if ((ret = av_packet_ref(pkt, &is->video_st->attached_pic)) < 0) // 附加图片数据包引用到 pkt 中。
                    goto fail; // 如果引用失败（ret < 0），跳转到 fail 标签处理错误。
                //* 如果有，则将其放入视频队列
                packet_queue_put(&is->videoq, pkt);
                // 向视频队列中放入一个空包（null packet），用于标记数据流的结束。
                packet_queue_put_nullpacket(&is->videoq, pkt, is->video_stream); // 空包通常用于刷新解码器或标记流的结束。
            }
            // 在处理完附加图片后，将该标志重置为 0，表示请求已处理完毕。
            is->queue_attachments_req = 0;
        }

        /* if the queue are full, no need to read more */
        //* 检查音频、视频和字幕队列的状态：
            // 如果队列已满或数据足够，则暂停读取数据，等待一段时间后再继续。
            // 如果队列未满且数据不足，则继续读取数据。检查是否需要读取更多数据包
        if ( infinite_buffer < 1 && // 非无限缓冲，需要检查队列状态。
            (is->audioq.size + is->videoq.size + is->subtitleq.size > MAX_QUEUE_SIZE // 检查音频、视频和字幕队列的总大小是否超过最大队列大小（MAX_QUEUE_SIZE）
                // 检查某个流的数据包是否足够
                || (stream_has_enough_packets(is->audio_st, is->audio_stream, &is->audioq) &&
                    stream_has_enough_packets(is->video_st, is->video_stream, &is->videoq) &&
                    stream_has_enough_packets(is->subtitle_st, is->subtitle_stream, &is->subtitleq))) ) {
            /* wait 10 ms */
            SDL_LockMutex(wait_mutex);
            SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
            SDL_UnlockMutex(wait_mutex);
            continue;
        }

        //* 处理播放结束逻辑的一部分。它检查播放器是否处于播放结束状态，并根据配置决定是否循环播放或自动退出。
        // 检查播放结束条件, 条件都满足表示播放已经结束。
        if ( !is->paused && // 检查播放器是否未处于暂停状态。
            (!is->audio_st || (is->auddec.finished == is->audioq.serial && frame_queue_nb_remaining(&is->sampq) == 0)) &&
                /* !is->audio_st：如果音频流不存在，则跳过音频流的检查。 */
                // is->auddec.finished == is->audioq.serial：检查音频解码器是否已完成解码，并且解码器的序列号与音频队列的序列号一致。
                // frame_queue_nb_remaining(&is->sampq) == 0：检查音频帧队列中是否没有剩余的帧。
            (!is->video_st || (is->viddec.finished == is->videoq.serial && frame_queue_nb_remaining(&is->pictq) == 0))) {
            if (loop != 1 && (!loop || --loop)) {
                // loop != 1：如果 loop 不等于 1，表示需要检查是否启用循环播放。
                /* !loop || --loop：如果 loop 为 0，表示无限循环播放。 如果 loop 大于 1，表示有限次循环播放，每次循环后 loop 减 1。*/
                //* 将播放位置跳转到指定的开始时间（start_time）
                // 如果 start_time 是无效值（AV_NOPTS_VALUE），则跳转到 0（文件开头）。
                stream_seek(is, start_time != AV_NOPTS_VALUE ? start_time : 0, 0, 0);
            } else if (autoexit) { // 如果设置了自动退出，则在播放结束后返回eof并进行清理。
                ret = AVERROR_EOF; 
                goto fail;
            }
        }
        
        //* 从输入流中读取数据包。
        ret = av_read_frame(ic, pkt);
        if ( ret < 0 ) {//* 返回值小于 0，表示读取出错或到达文件末尾。
            if ((ret == AVERROR_EOF || avio_feof(ic->pb)) && !is->eof) { //* 处理 EOF（文件结束）情况，向队列中添加空包以指示流已结束。
                // ret == AVERROR_EOF：表示 FFmpeg 返回了文件结束错误。
                // avio_feof(ic->pb)：表示输入流的底层 I/O 层检测到文件结束。
                /* !is->eof：确保当前未处于 EOF 状态。*/
                //* 向各个流发送空包：用于标记流的结束，通知解码器刷新缓冲区。
                if (is->video_stream >= 0)
                    packet_queue_put_nullpacket(&is->videoq, pkt, is->video_stream);
                if (is->audio_stream >= 0)
                    packet_queue_put_nullpacket(&is->audioq, pkt, is->audio_stream);
                if (is->subtitle_stream >= 0)
                    packet_queue_put_nullpacket(&is->subtitleq, pkt, is->subtitle_stream);
                //* 更新 eof 标识。表示已到达文件末尾。
                is->eof = 1; 
            }
            //* 读取错误
            if (ic->pb && ic->pb->error) { // 如果输入流的底层 I/O 层（ic->pb）发生错误（ic->pb->error），则进行错误处理。
                //* 如果发生读取错误且 autoexit 被设置，则跳转到清理并退出的部分。
                if (autoexit)
                    goto fail;
                else
                    break;
            }
            //* 如果不自动退出，等待重试
            SDL_LockMutex(wait_mutex);
            SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
            SDL_UnlockMutex(wait_mutex);
            continue;
        } else {
            //* 读取成功（ret >= 0），将 is->eof 重置为 0，表示未到达文件末尾。
            is->eof = 0;
        }
        
        /* check if packet is in play range specified by user, then queue, otherwise discard */
        //* 检查从输入流中读取的数据包（AVPacket）是否在用户指定的播放范围内，
        //* 并根据数据包的流索引将其放入相应的队列中。
        //* 如果数据包不在播放范围内，则直接释放它。
        stream_start_time = ic->streams[pkt->stream_index]->start_time; // 获取当前流（pkt->stream_index）的开始时间（start_time）
        pkt_ts = pkt->pts == AV_NOPTS_VALUE ? pkt->dts : pkt->pts; // 如果数据包的显示时间戳（pts）无效（AV_NOPTS_VALUE），则使用解码时间戳（dts）作为时间戳。
        //* 检查数据包是否在播放范围内
        pkt_in_play_range = duration == AV_NOPTS_VALUE || // 如果播放时长（duration）无效（AV_NOPTS_VALUE），则认为数据包在播放范围内。
            (pkt_ts - (stream_start_time != AV_NOPTS_VALUE ? stream_start_time : 0))  
            *
            av_q2d(ic->streams[pkt->stream_index]->time_base) // 将数据包的时间戳（pkt_ts）减去流的开始时间（stream_start_time)
            -
            (double)(start_time != AV_NOPTS_VALUE ? start_time : 0) / 1000000 // 用户指定的开始时间（start_time）转换为秒
            <= ((double)duration / 1000000); // 如果数据包的相对时间小于或等于播放时长（duration），则认为数据包在播放范围内。

        //* 如果数据包在播放范围内, 根据数据包的流索引，放入相应的队列。
        if (pkt->stream_index == is->audio_stream && pkt_in_play_range) {
            packet_queue_put(&is->audioq, pkt);
        } else if (pkt->stream_index == is->video_stream && pkt_in_play_range
                   && !(is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC)) {
            packet_queue_put(&is->videoq, pkt);
        } else if (pkt->stream_index == is->subtitle_stream && pkt_in_play_range) {
            packet_queue_put(&is->subtitleq, pkt);
        } else {
            //* 如果数据包不在播放范围内，调用 av_packet_unref 释放该数据包。
            av_packet_unref(pkt);
        }
    }

    ret = 0;
    
fail:
    //* 如果发生错误，则发送退出事件以终止程序。
    
    //* 关闭输入流
    if ( ic && !is->ic )
        avformat_close_input(&ic);
    //* 释放数据包指针
    av_packet_free(&pkt);
    //* 如果返回值不是 0（表示发生错误），则生成退出事件，通知 SDL 退出事件处理循环。
    if (ret != 0) {
        SDL_Event event;

        event.type = FF_QUIT_EVENT;
        event.user.data1 = is;
        SDL_PushEvent(&event);
    }
    //* 销毁互斥锁
    SDL_DestroyMutex(wait_mutex);
    return 0;
}

//* 为媒体播放器或相关应用程序设置和配置视频播放环境。
//* 返回一个指向 VideoState 结构的指针，如果打开失败则返回 NULL。
static VideoState *stream_open(const char *filename,
                               const AVInputFormat *iformat)
{
    VideoState *is;

    //*  分配内存并初始化为零
    is = av_mallocz(sizeof(VideoState));
    if (!is)
        return NULL;
    //* 初始化流索引
    is->last_video_stream = is->video_stream = -1;
    is->last_audio_stream = is->audio_stream = -1;
    is->last_subtitle_stream = is->subtitle_stream = -1;
    
    //* 复制传入的文件名，如果失败则进入失败处理
    is->filename = av_strdup(filename);
    if (!is->filename)
        goto fail;
    //* 初始化格式信息和左上顶点
    is->iformat = iformat;
    is->ytop    = 0;
    is->xleft   = 0;

    /* start video display */
    //* 初始化视频、字幕和音频的帧队列，创建时传入各个队列的大小等参数。如果任何一个初始化失败，则处理失败。
    if ( frame_queue_init(&is->pictq, &is->videoq, VIDEO_PICTURE_QUEUE_SIZE, 1) < 0 )
        goto fail;
    if (frame_queue_init(&is->subpq, &is->subtitleq, SUBPICTURE_QUEUE_SIZE, 0) < 0)
        goto fail;
    if (frame_queue_init(&is->sampq, &is->audioq, SAMPLE_QUEUE_SIZE, 1) < 0)
        goto fail;
    //* 初始化视频、音频和字幕的包队列。如果失败，处理失败。
    if (packet_queue_init(&is->videoq) < 0 ||
        packet_queue_init(&is->audioq) < 0 ||
        packet_queue_init(&is->subtitleq) < 0)
        goto fail;

    //* 创建一个条件变量，用于read线程和事件线程（主线程）间的同步
    if ( !(is->continue_read_thread = SDL_CreateCond()) ) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        goto fail;
    }

    //* 初始化视频、音频和外部时钟。
    init_clock(&is->vidclk, &is->videoq.serial);
    init_clock(&is->audclk, &is->audioq.serial);
    init_clock(&is->extclk, &is->extclk.serial);
    is->audio_clock_serial = -1;

    //* 设置初始音量，并处理超出范围的情况，确保音量在合法范围内（0-100）。
    if ( startup_volume < 0 )
        av_log(NULL, AV_LOG_WARNING, "-volume=%d < 0, setting to 0\n", startup_volume);
    if (startup_volume > 100)
        av_log(NULL, AV_LOG_WARNING, "-volume=%d > 100, setting to 100\n", startup_volume);
    startup_volume = av_clip(startup_volume, 0, 100);
    startup_volume = av_clip(SDL_MIX_MAXVOLUME * startup_volume / 100, 0, SDL_MIX_MAXVOLUME);
    is->audio_volume = startup_volume;
    is->muted = 0;

    //* 设置同步类型
    is->av_sync_type = av_sync_type;
    //* 创建read线程
    is->read_tid = SDL_CreateThread(read_thread, "read_thread", is);
    if (!is->read_tid) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateThread(): %s\n", SDL_GetError());

    //* 错误处理，关闭输入流，返回NULL
fail:
        stream_close(is);
        return NULL;
    }
    //* 成功返回输入流
    return is;
}

static void stream_cycle_channel(VideoState *is, int codec_type)
{
    AVFormatContext *ic = is->ic;
    int start_index, stream_index;
    int old_index;
    AVStream *st;
    AVProgram *p = NULL;
    int nb_streams = is->ic->nb_streams;

    if (codec_type == AVMEDIA_TYPE_VIDEO) {
        start_index = is->last_video_stream;
        old_index = is->video_stream;
    } else if (codec_type == AVMEDIA_TYPE_AUDIO) {
        start_index = is->last_audio_stream;
        old_index = is->audio_stream;
    } else {
        start_index = is->last_subtitle_stream;
        old_index = is->subtitle_stream;
    }
    stream_index = start_index;

    if (codec_type != AVMEDIA_TYPE_VIDEO && is->video_stream != -1) {
        p = av_find_program_from_stream(ic, NULL, is->video_stream);
        if (p) {
            nb_streams = p->nb_stream_indexes;
            for (start_index = 0; start_index < nb_streams; start_index++)
                if (p->stream_index[start_index] == stream_index)
                    break;
            if (start_index == nb_streams)
                start_index = -1;
            stream_index = start_index;
        }
    }

    for (;;) {
        if (++stream_index >= nb_streams)
        {
            if (codec_type == AVMEDIA_TYPE_SUBTITLE)
            {
                stream_index = -1;
                is->last_subtitle_stream = -1;
                goto the_end;
            }
            if (start_index == -1)
                return;
            stream_index = 0;
        }
        if (stream_index == start_index)
            return;
        st = is->ic->streams[p ? p->stream_index[stream_index] : stream_index];
        if (st->codecpar->codec_type == codec_type) {
            /* check that parameters are OK */
            switch (codec_type) {
            case AVMEDIA_TYPE_AUDIO:
                if (st->codecpar->sample_rate != 0 &&
                    st->codecpar->ch_layout.nb_channels != 0)
                    goto the_end;
                break;
            case AVMEDIA_TYPE_VIDEO:
            case AVMEDIA_TYPE_SUBTITLE:
                goto the_end;
            default:
                break;
            }
        }
    }
 the_end:
    if (p && stream_index != -1)
        stream_index = p->stream_index[stream_index];
    av_log(NULL, AV_LOG_INFO, "Switch %s stream from #%d to #%d\n",
           av_get_media_type_string(codec_type),
           old_index,
           stream_index);

    stream_component_close(is, old_index);
    stream_component_open(is, stream_index);
}


static void toggle_full_screen(VideoState *is)
{
    is_full_screen = !is_full_screen;
    SDL_SetWindowFullscreen(window, is_full_screen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
}

static void toggle_audio_display(VideoState *is)
{
    int next = is->show_mode;
    do {
        next = (next + 1) % SHOW_MODE_NB;
    } while (next != is->show_mode && (next == SHOW_MODE_VIDEO && !is->video_st || next != SHOW_MODE_VIDEO && !is->audio_st));
    if (is->show_mode != next) {
        is->force_refresh = 1;
        is->show_mode = next;
    }
}

//* 等待和处理 SDL 事件，同时控制视频刷新逻辑。 
static void refresh_loop_wait_event(VideoState *is, SDL_Event *event) {
    double remaining_time = 0.0;
    SDL_PumpEvents(); // 更新 SDL 的事件队列，确保可以获取最新的事件。
    //* 如果没有事件，则继续循环；如果有事件，则退出循环。
    // 没有事件发生时，跟随刷新率的频次进行事件检查和刷新；有事件时，退出处理事件，由外部处理完事件后再次驱动进去刷新周期检查
    while (!SDL_PeepEvents(event, 1, SDL_GETEVENT, SDL_FIRSTEVENT, SDL_LASTEVENT)) { // 检查事件队列中是否有新的事件。  SDL_FIRSTEVENT 和 SDL_LASTEVENT：事件类型范围，表示获取所有类型的事件。
        // 如果光标未隐藏且超过延迟时间，则调用 SDL_ShowCursor(0) 隐藏光标，并设置 cursor_hidden = 1。
        if (!cursor_hidden && av_gettime_relative() - cursor_last_shown > CURSOR_HIDE_DELAY) { // cursor_hidden：标记光标是否已隐藏。  cursor_last_shown：记录光标上次显示的时间。 CURSOR_HIDE_DELAY：光标隐藏的延迟时间（单位：微秒）。
            SDL_ShowCursor(0);
            cursor_hidden = 1;
        }

        //刷新率控制: 下一次刷新的剩余时间（单位：秒）。
        if (remaining_time > 0.0) // emaining_time：下一次刷新的剩余时间（单位：秒）。
            av_usleep((int64_t)(remaining_time * 1000000.0));
        remaining_time = REFRESH_RATE;

        // 视频刷新逻辑：非禁用显式、非暂停或强制刷新时，进行视频刷新
        if (is->show_mode != SHOW_MODE_NONE && (!is->paused || is->force_refresh))
            video_refresh(is, &remaining_time);

        // 处理完光标、图刷后，再次更新 SDL 的事件队列，及时获取最新的事件。
        SDL_PumpEvents();
    }
}

static void seek_chapter(VideoState *is, int incr)
{
    int64_t pos = get_master_clock(is) * AV_TIME_BASE;
    int i;

    if (!is->ic->nb_chapters)
        return;

    /* find the current chapter */
    for (i = 0; i < is->ic->nb_chapters; i++) {
        AVChapter *ch = is->ic->chapters[i];
        if (av_compare_ts(pos, AV_TIME_BASE_Q, ch->start, ch->time_base) < 0) {
            i--;
            break;
        }
    }

    i += incr;
    i = FFMAX(i, 0);
    if (i >= is->ic->nb_chapters)
        return;

    av_log(NULL, AV_LOG_VERBOSE, "Seeking to chapter %d.\n", i);
    stream_seek(is, av_rescale_q(is->ic->chapters[i]->start, is->ic->chapters[i]->time_base,
                                 AV_TIME_BASE_Q), 0, 0);
}

/* handle an event sent by the GUI */
//* 利用 SDL（Simple DirectMedia Layer）来处理与视频播放相关的事件。主要目的是管理用户通过键盘和鼠标输入进行的交互，从而实现视频播放的各种控制。
static void event_loop(VideoState *cur_stream)
{
    SDL_Event event;
    double incr, pos, frac;

    //* loop
    for ( ;;) {
        double x;
        refresh_loop_wait_event(cur_stream, &event);
        //* event handle
        switch ( event.type ) {
        //* 键盘输入处理
        case SDL_KEYDOWN:
            //* 按 ESC 键、q 键或者通过预定义的退出变量 （exit_on_keydown）来退出应用程序
            if ( exit_on_keydown || event.key.keysym.sym == SDLK_ESCAPE || event.key.keysym.sym == SDLK_q ) {
                do_exit(cur_stream);
                break;
            }
            // If we don't yet have a window, skip all key events, because read_thread might still be initializing...
            //* 没有窗口就忽略所有键盘事件
            if ( !cur_stream->width )
                continue;
            //* 键盘事件
            switch ( event.key.keysym.sym ) {
            //* f 键可以在全屏和窗口模式之间切换
            case SDLK_f:
                toggle_full_screen(cur_stream);
                cur_stream->force_refresh = 1;
                break;
            //* 按 p 或 SPACE 键可以切换暂停/播放状态
            case SDLK_p:
            case SDLK_SPACE:
                toggle_pause(cur_stream);
                break;
            //* 按 m 键可以静音或取消静音。
            case SDLK_m:
                toggle_mute(cur_stream);
                break;
            //* KP_MULTIPLY（小键盘的乘号）或 0 键增加音量。
            case SDLK_KP_MULTIPLY:
            case SDLK_0:
                update_volume(cur_stream, 1, SDL_VOLUME_STEP);
                break;
            //* KP_DIVIDE（小键盘的除号）或 9 键降低音量
            case SDLK_KP_DIVIDE:
            case SDLK_9:
                update_volume(cur_stream, -1, SDL_VOLUME_STEP);
                break;
            //* 逐帧播放，按 s 键可以跳到下一帧。
            case SDLK_s: // S: Step to next frame
                step_to_next_frame(cur_stream);
                break;
            //* a、v、c、t 和 w 键可以在音频、视频和字幕频道之间切换。
            case SDLK_a:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_AUDIO);
                break;
            case SDLK_v:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_VIDEO);
                break;
            case SDLK_c:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_VIDEO);
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_AUDIO);
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_SUBTITLE);
                break;
            case SDLK_t:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_SUBTITLE);
                break;
            case SDLK_w:
                if (cur_stream->show_mode == SHOW_MODE_VIDEO && cur_stream->vfilter_idx < nb_vfilters - 1) {
                    if (++cur_stream->vfilter_idx >= nb_vfilters)
                        cur_stream->vfilter_idx = 0;
                } else {
                    cur_stream->vfilter_idx = 0;
                    toggle_audio_display(cur_stream);
                }
                break;
            //* 使用 PAGEUP 和 PAGEDOWN 键在可用的章节之间切换。
            case SDLK_PAGEUP:
                if (cur_stream->ic->nb_chapters <= 1) {
                    incr = 600.0;
                    goto do_seek;
                }
                seek_chapter(cur_stream, 1);
                break;
            case SDLK_PAGEDOWN:
                if (cur_stream->ic->nb_chapters <= 1) {
                    incr = -600.0;
                    goto do_seek;
                }
                seek_chapter(cur_stream, -1);
                break;
            //* 箭头键（←、→、↑、↓）可用于在视频中前后寻址。增加或减少的时间间隔由 seek_interval 指定（如果有）或使用标准值。
            case SDLK_LEFT:
                incr = seek_interval ? -seek_interval : -10.0;
                goto do_seek;
            case SDLK_RIGHT:
                incr = seek_interval ? seek_interval : 10.0;
                goto do_seek;
            case SDLK_UP:
                incr = 60.0;
                goto do_seek;
            case SDLK_DOWN:
                incr = -60.0;
            do_seek:
                    if (seek_by_bytes) {
                        pos = -1;
                        if (pos < 0 && cur_stream->video_stream >= 0)
                            pos = frame_queue_last_pos(&cur_stream->pictq);
                        if (pos < 0 && cur_stream->audio_stream >= 0)
                            pos = frame_queue_last_pos(&cur_stream->sampq);
                        if (pos < 0)
                            pos = avio_tell(cur_stream->ic->pb);
                        if (cur_stream->ic->bit_rate)
                            incr *= cur_stream->ic->bit_rate / 8.0;
                        else
                            incr *= 180000.0;
                        pos += incr;
                        stream_seek(cur_stream, pos, incr, 1);
                    } else {
                        pos = get_master_clock(cur_stream);
                        if (isnan(pos))
                            pos = (double)cur_stream->seek_pos / AV_TIME_BASE;
                        pos += incr;
                        if (cur_stream->ic->start_time != AV_NOPTS_VALUE && pos < cur_stream->ic->start_time / (double)AV_TIME_BASE)
                            pos = cur_stream->ic->start_time / (double)AV_TIME_BASE;
                        stream_seek(cur_stream, (int64_t)(pos * AV_TIME_BASE), (int64_t)(incr * AV_TIME_BASE), 0);
                    }
                break;
            default:
                break;
            }
            break;
        //*  鼠标输入处理
        case SDL_MOUSEBUTTONDOWN:
            if (exit_on_mousedown) {
                do_exit(cur_stream);
                break;
            }
            if ( event.button.button == SDL_BUTTON_LEFT ) {
                //* 快速双击左键会切换全屏。
                static int64_t last_mouse_left_click = 0;
                if (av_gettime_relative() - last_mouse_left_click <= 500000) {
                    toggle_full_screen(cur_stream);
                    cur_stream->force_refresh = 1;
                    last_mouse_left_click = 0;
                } else {
                    last_mouse_left_click = av_gettime_relative();
                }
            }
        //* 鼠标移动处理
        case SDL_MOUSEMOTION:
            //* 如果鼠标被移动且光标被隐藏，则会显示光标。
            if ( cursor_hidden ) {
                SDL_ShowCursor(1);
                cursor_hidden = 0;
            }
            //* 根据鼠标按键的状态（按下或移动）来决定是否进行视频的寻址。
            cursor_last_shown = av_gettime_relative();
            if (event.type == SDL_MOUSEBUTTONDOWN) {
                if (event.button.button != SDL_BUTTON_RIGHT)
                    break;
                x = event.button.x;
            } else {
                if (!(event.motion.state & SDL_BUTTON_RMASK))
                    break;
                x = event.motion.x;
            }
            if (seek_by_bytes || cur_stream->ic->duration <= 0) {
                uint64_t size =  avio_size(cur_stream->ic->pb);
                stream_seek(cur_stream, size*x/cur_stream->width, 0, 1);
            } else {
                int64_t ts;
                int ns, hh, mm, ss;
                int tns, thh, tmm, tss;
                tns  = cur_stream->ic->duration / 1000000LL;
                thh  = tns / 3600;
                tmm  = (tns % 3600) / 60;
                tss  = (tns % 60);
                frac = x / cur_stream->width;
                ns   = frac * tns;
                hh   = ns / 3600;
                mm   = (ns % 3600) / 60;
                ss   = (ns % 60);
                av_log(NULL, AV_LOG_INFO,
                        "Seek to %2.0f%% (%2d:%02d:%02d) of total duration (%2d:%02d:%02d)       \n", frac*100,
                        hh, mm, ss, thh, tmm, tss);
                ts = frac * cur_stream->ic->duration;
                if (cur_stream->ic->start_time != AV_NOPTS_VALUE)
                    ts += cur_stream->ic->start_time;
                stream_seek(cur_stream, ts, 0, 0);
            }
            break;
        //* 窗口事件
        case SDL_WINDOWEVENT:
            switch ( event.window.event ) {
            //* 窗口大小发生变化
            case SDL_WINDOWEVENT_SIZE_CHANGED:
                //* 会更新流的宽度和高度
                screen_width = cur_stream->width = event.window.data1;
                screen_height = cur_stream->height = event.window.data2;
                //* 销毁纹理（如果存在）
                if ( cur_stream->vis_texture ) {
                    SDL_DestroyTexture(cur_stream->vis_texture);
                    cur_stream->vis_texture = NULL;
                }
                //* 在适用时调整渲染器的大小。
                if ( vk_renderer )
                    vk_renderer_resize(vk_renderer, screen_width, screen_height);
             //* 窗口被暴露，将强制刷新显示
            case SDL_WINDOWEVENT_EXPOSED:
                cur_stream->force_refresh = 1;
            }
            break;
        //* 退出处理
        //* 检查 SDL_QUIT 和自定义的退出事件（FF_QUIT_EVENT）
        case SDL_QUIT:
        case FF_QUIT_EVENT:
            do_exit(cur_stream);
            break;
        default:
            break;
        }
    }
}

static int opt_width(void *optctx, const char *opt, const char *arg)
{
    double num;
    int ret = parse_number(opt, arg, OPT_TYPE_INT64, 1, INT_MAX, &num);
    if (ret < 0)
        return ret;

    screen_width = num;
    return 0;
}

static int opt_height(void *optctx, const char *opt, const char *arg)
{
    double num;
    int ret = parse_number(opt, arg, OPT_TYPE_INT64, 1, INT_MAX, &num);
    if (ret < 0)
        return ret;

    screen_height = num;
    return 0;
}

static int opt_format(void *optctx, const char *opt, const char *arg)
{
    file_iformat = av_find_input_format(arg);
    if (!file_iformat) {
        av_log(NULL, AV_LOG_FATAL, "Unknown input format: %s\n", arg);
        return AVERROR(EINVAL);
    }
    return 0;
}

static int opt_sync(void *optctx, const char *opt, const char *arg)
{
    if (!strcmp(arg, "audio"))
        av_sync_type = AV_SYNC_AUDIO_MASTER;
    else if (!strcmp(arg, "video"))
        av_sync_type = AV_SYNC_VIDEO_MASTER;
    else if (!strcmp(arg, "ext"))
        av_sync_type = AV_SYNC_EXTERNAL_CLOCK;
    else {
        av_log(NULL, AV_LOG_ERROR, "Unknown value for %s: %s\n", opt, arg);
        exit(1);
    }
    return 0;
}

static int opt_show_mode(void *optctx, const char *opt, const char *arg)
{
    show_mode = !strcmp(arg, "video") ? SHOW_MODE_VIDEO :
                !strcmp(arg, "waves") ? SHOW_MODE_WAVES :
                !strcmp(arg, "rdft" ) ? SHOW_MODE_RDFT  : SHOW_MODE_NONE;

    if (show_mode == SHOW_MODE_NONE) {
        double num;
        int ret = parse_number(opt, arg, OPT_TYPE_INT, 0, SHOW_MODE_NB-1, &num);
        if (ret < 0)
            return ret;
        show_mode = num;
    }
    return 0;
}

static int opt_input_file(void *optctx, const char *filename)
{
    if (input_filename) {
        av_log(NULL, AV_LOG_FATAL,
               "Argument '%s' provided as input filename, but '%s' was already specified.\n",
                filename, input_filename);
        return AVERROR(EINVAL);
    }
    if (!strcmp(filename, "-"))
        filename = "fd:";
    input_filename = av_strdup(filename);
    if (!input_filename)
        return AVERROR(ENOMEM);

    return 0;
}

static int opt_codec(void *optctx, const char *opt, const char *arg)
{
   const char *spec = strchr(opt, ':');
   const char **name;
   if (!spec) {
       av_log(NULL, AV_LOG_ERROR,
              "No media specifier was specified in '%s' in option '%s'\n",
               arg, opt);
       return AVERROR(EINVAL);
   }
   spec++;

   switch (spec[0]) {
   case 'a' : name = &audio_codec_name;    break;
   case 's' : name = &subtitle_codec_name; break;
   case 'v' : name = &video_codec_name;    break;
   default:
       av_log(NULL, AV_LOG_ERROR,
              "Invalid media specifier '%s' in option '%s'\n", spec, opt);
       return AVERROR(EINVAL);
   }

   av_freep(name);
   *name = av_strdup(arg);
   return *name ? 0 : AVERROR(ENOMEM);
}

static int dummy;

static const OptionDef options[] = {
    CMDUTILS_COMMON_OPTIONS
    { "x",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_width }, "force displayed width", "width" },
    { "y",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_height }, "force displayed height", "height" },
    { "fs",                 OPT_TYPE_BOOL,            0, { &is_full_screen }, "force full screen" },
    { "an",                 OPT_TYPE_BOOL,            0, { &audio_disable }, "disable audio" },
    { "vn",                 OPT_TYPE_BOOL,            0, { &video_disable }, "disable video" },
    { "sn",                 OPT_TYPE_BOOL,            0, { &subtitle_disable }, "disable subtitling" },
    { "ast",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_AUDIO] }, "select desired audio stream", "stream_specifier" },
    { "vst",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_VIDEO] }, "select desired video stream", "stream_specifier" },
    { "sst",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_SUBTITLE] }, "select desired subtitle stream", "stream_specifier" },
    { "ss",                 OPT_TYPE_TIME,            0, { &start_time }, "seek to a given position in seconds", "pos" },
    { "t",                  OPT_TYPE_TIME,            0, { &duration }, "play  \"duration\" seconds of audio/video", "duration" },
    { "bytes",              OPT_TYPE_INT,             0, { &seek_by_bytes }, "seek by bytes 0=off 1=on -1=auto", "val" },
    { "seek_interval",      OPT_TYPE_FLOAT,           0, { &seek_interval }, "set seek interval for left/right keys, in seconds", "seconds" },
    { "nodisp",             OPT_TYPE_BOOL,            0, { &display_disable }, "disable graphical display" },
    { "noborder",           OPT_TYPE_BOOL,            0, { &borderless }, "borderless window" },
    { "alwaysontop",        OPT_TYPE_BOOL,            0, { &alwaysontop }, "window always on top" },
    { "volume",             OPT_TYPE_INT,             0, { &startup_volume}, "set startup volume 0=min 100=max", "volume" },
    { "f",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_format }, "force format", "fmt" },
    { "stats",              OPT_TYPE_BOOL,   OPT_EXPERT, { &show_status }, "show status", "" },
    { "fast",               OPT_TYPE_BOOL,   OPT_EXPERT, { &fast }, "non spec compliant optimizations", "" },
    { "genpts",             OPT_TYPE_BOOL,   OPT_EXPERT, { &genpts }, "generate pts", "" },
    { "drp",                OPT_TYPE_INT,    OPT_EXPERT, { &decoder_reorder_pts }, "let decoder reorder pts 0=off 1=on -1=auto", ""},
    { "lowres",             OPT_TYPE_INT,    OPT_EXPERT, { &lowres }, "", "" },
    { "sync",               OPT_TYPE_FUNC, OPT_FUNC_ARG | OPT_EXPERT, { .func_arg = opt_sync }, "set audio-video sync. type (type=audio/video/ext)", "type" },
    { "autoexit",           OPT_TYPE_BOOL,   OPT_EXPERT, { &autoexit }, "exit at the end", "" },
    { "exitonkeydown",      OPT_TYPE_BOOL,   OPT_EXPERT, { &exit_on_keydown }, "exit on key down", "" },
    { "exitonmousedown",    OPT_TYPE_BOOL,   OPT_EXPERT, { &exit_on_mousedown }, "exit on mouse down", "" },
    { "loop",               OPT_TYPE_INT,    OPT_EXPERT, { &loop }, "set number of times the playback shall be looped", "loop count" },
    { "framedrop",          OPT_TYPE_BOOL,   OPT_EXPERT, { &framedrop }, "drop frames when cpu is too slow", "" },
    { "infbuf",             OPT_TYPE_BOOL,   OPT_EXPERT, { &infinite_buffer }, "don't limit the input buffer size (useful with realtime streams)", "" },
    { "window_title",       OPT_TYPE_STRING,          0, { &window_title }, "set window title", "window title" },
    { "left",               OPT_TYPE_INT,    OPT_EXPERT, { &screen_left }, "set the x position for the left of the window", "x pos" },
    { "top",                OPT_TYPE_INT,    OPT_EXPERT, { &screen_top }, "set the y position for the top of the window", "y pos" },
    { "vf",                 OPT_TYPE_FUNC, OPT_FUNC_ARG | OPT_EXPERT, { .func_arg = opt_add_vfilter }, "set video filters", "filter_graph" },
    { "af",                 OPT_TYPE_STRING,          0, { &afilters }, "set audio filters", "filter_graph" },
    { "rdftspeed",          OPT_TYPE_INT, OPT_AUDIO | OPT_EXPERT, { &rdftspeed }, "rdft speed", "msecs" },
    { "showmode",           OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_show_mode}, "select show mode (0 = video, 1 = waves, 2 = RDFT)", "mode" },
    { "i",                  OPT_TYPE_BOOL,            0, { &dummy}, "read specified file", "input_file"},
    { "codec",              OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_codec}, "force decoder", "decoder_name" },
    { "acodec",             OPT_TYPE_STRING, OPT_EXPERT, {    &audio_codec_name }, "force audio decoder",    "decoder_name" },
    { "scodec",             OPT_TYPE_STRING, OPT_EXPERT, { &subtitle_codec_name }, "force subtitle decoder", "decoder_name" },
    { "vcodec",             OPT_TYPE_STRING, OPT_EXPERT, {    &video_codec_name }, "force video decoder",    "decoder_name" },
    { "autorotate",         OPT_TYPE_BOOL,            0, { &autorotate }, "automatically rotate video", "" },
    { "find_stream_info",   OPT_TYPE_BOOL, OPT_INPUT | OPT_EXPERT, { &find_stream_info },
        "read and decode the streams to fill missing information with heuristics" },
    { "filter_threads",     OPT_TYPE_INT,    OPT_EXPERT, { &filter_nbthreads }, "number of filter threads per graph" },
    { "enable_vulkan",      OPT_TYPE_BOOL,            0, { &enable_vulkan }, "enable vulkan renderer" },
    { "vulkan_params",      OPT_TYPE_STRING, OPT_EXPERT, { &vulkan_params }, "vulkan configuration using a list of key=value pairs separated by ':'" },
    { "hwaccel",            OPT_TYPE_STRING, OPT_EXPERT, { &hwaccel }, "use HW accelerated decoding" },
    { NULL, },
};

static void show_usage(void)
{
    av_log(NULL, AV_LOG_INFO, "Simple media player\n");
    av_log(NULL, AV_LOG_INFO, "usage: %s [options] input_file\n", program_name);
    av_log(NULL, AV_LOG_INFO, "\n");
}

void show_help_default(const char *opt, const char *arg)
{
    av_log_set_callback(log_callback_help);
    show_usage();
    show_help_options(options, "Main options:", 0, OPT_EXPERT);
    show_help_options(options, "Advanced options:", OPT_EXPERT, 0);
    printf("\n");
    show_help_children(avcodec_get_class(), AV_OPT_FLAG_DECODING_PARAM);
    show_help_children(avformat_get_class(), AV_OPT_FLAG_DECODING_PARAM);
    show_help_children(avfilter_get_class(), AV_OPT_FLAG_FILTERING_PARAM);
    printf("\nWhile playing:\n"
           "q, ESC              quit\n"
           "f                   toggle full screen\n"
           "p, SPC              pause\n"
           "m                   toggle mute\n"
           "9, 0                decrease and increase volume respectively\n"
           "/, *                decrease and increase volume respectively\n"
           "a                   cycle audio channel in the current program\n"
           "v                   cycle video channel\n"
           "t                   cycle subtitle channel in the current program\n"
           "c                   cycle program\n"
           "w                   cycle video filters or show modes\n"
           "s                   activate frame-step mode\n"
           "left/right          seek backward/forward 10 seconds or to custom interval if -seek_interval is set\n"
           "down/up             seek backward/forward 1 minute\n"
           "page down/page up   seek backward/forward 10 minutes\n"
           "right mouse click   seek to percentage in file corresponding to fraction of width\n"
           "left double-click   toggle full screen\n"
           );
}

/* Called from the main */
int main(int argc, char **argv)
{
    int flags, ret;
    VideoState *is;

    //* 初始化动态库
    init_dynload();

    //* 日志设置
    av_log_set_flags(AV_LOG_SKIP_REPEATED);
    parse_loglevel(argc, argv, options);

    /* register all codecs, demux and protocols */
    //* CONFIG_AVDEVICE 是 FFmpeg 项目中的一个预处理宏，用于启用设备支持（即输入/输出设备的支持功能）。
    //* 比如访问摄像头或麦克风等输入/输出设备，你需要在编译时通过 --enable-avdevice 选项来定义 CONFIG_AVDEVICE
    //*     编译 FFmpeg 时，. / configure --enable-avdevice
    //* 一些特定平台可能不支持设备功能，因此在这些平台上，CONFIG_AVDEVICE 可能不会被定义。
    //*     对于嵌入式系统或某些精简版的 FFmpeg 构建，可能会关闭设备支持，以减小体积和复杂性。
#if CONFIG_AVDEVICE
    avdevice_register_all();
#endif
    avformat_network_init();

    //* ctrl+c ctrl+z 直接退出
    signal(SIGINT, sigterm_handler); /* Interrupt (ANSI).    */
    signal(SIGTERM, sigterm_handler); /* Termination (ANSI).  */

    //* 打印输出FFmpeg版本信息（编译时间，编译选项，类库信息等）。
    //* 就是ffplay播放时，媒体信息上面那一坨配置和库版本信息
    show_banner(argc, argv, options);

    //* 解析输入的选项，未识别为选项的参数为输入的播放路径
    ret = parse_options(NULL, argc, argv, options, opt_input_file);
    if (ret < 0)
        exit(ret == AVERROR_EXIT ? 0 : 1);

    //* 没有提供输入源，则打印使用方法
    if (!input_filename) {
        show_usage();
        av_log(NULL, AV_LOG_FATAL, "An input file must be specified\n");
        av_log(NULL, AV_LOG_FATAL,
               "Use -h to get full help or, even better, run 'man %s'\n", program_name);
        exit(1);
    }

    //* nodisp 
    if ( display_disable ) {
        video_disable = 1;
    }
    flags = SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER;
    //* an
    if (audio_disable)
        flags &= ~SDL_INIT_AUDIO;
    else {
        /* Try to work around an occasional ALSA buffer underflow issue when the
         * period size is NPOT due to ALSA resampling by forcing the buffer size. */
        //* 检查环境变量 SDL_AUDIO_ALSA_SET_BUFFER_SIZE 是否已设置。
        //* 处理 ALSA（Advanced Linux Sound Architecture）在缓冲区下溢时的一个潜在问题，特别是在使用非二次幂（NPOT）大小的缓冲区时。
        if ( !SDL_getenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE") )
            SDL_setenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE","1", 1);
    }
    //* vn
    if ( display_disable )
        flags &= ~SDL_INIT_VIDEO;

    //* 初始化SDL
    if ( SDL_Init(flags) ) {
        av_log(NULL, AV_LOG_FATAL, "Could not initialize SDL - %s\n", SDL_GetError());
        av_log(NULL, AV_LOG_FATAL, "(Did you set the DISPLAY variable?)\n");
        exit(1);
    }

    //* 禁用系统窗口事件和用户事件
    //* 可能是出于性能考虑，或者是因为程序在当前上下文中不需要处理这些事件。
    SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);
    SDL_EventState(SDL_USEREVENT, SDL_IGNORE);

    //* 创建一个SDL窗口，并根据不同的条件设置窗口的属性和渲染器。
    if ( !display_disable ) {
        //* 窗口初始化时是隐藏的。
        int flags = SDL_WINDOW_HIDDEN;

        //* SDL 2.0.5以上窗口置顶
        if ( alwaysontop )
#if SDL_VERSION_ATLEAST(2,0,5)
            flags |= SDL_WINDOW_ALWAYS_ON_TOP;
#else
            av_log(NULL, AV_LOG_WARNING, "Your SDL version doesn't support SDL_WINDOW_ALWAYS_ON_TOP. Feature will be inactive.\n");
#endif
        
        //* 无边框设置
        if ( borderless )
            flags |= SDL_WINDOW_BORDERLESS;
        else
            flags |= SDL_WINDOW_RESIZABLE;

        //*     通过 SDL_SetHint 设置了一个与 X11 合成器相关的提示，用于控制 SDL 应用程序是否绕过合成器。
#ifdef SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR
        SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "0");
#endif

        //* vulkan硬件加速配置
        //* 设置了硬件加速但没有手动开启vulkan，则自动开启vulkan渲染
        if ( hwaccel && !enable_vulkan ) {
            av_log(NULL, AV_LOG_INFO, "Enable vulkan renderer to support hwaccel %s\n", hwaccel);
            enable_vulkan = 1;
        }
        //* 如果开启vulkan，尝试获取vulkan渲染器
        if ( enable_vulkan ) {
            vk_renderer = vk_get_renderer();
            if ( vk_renderer ) {
                //* 获取成功且SDL 2.0.6 以上，SDL设置vulkan渲染
#if SDL_VERSION_ATLEAST(2, 0, 6)
                flags |= SDL_WINDOW_VULKAN;
#endif
            } else {
                av_log(NULL, AV_LOG_WARNING, "Doesn't support vulkan renderer, fallback to SDL renderer\n");
                enable_vulkan = 0;
            }
        }

        //* 通过前述配置创建SDL窗口
        window = SDL_CreateWindow(program_name, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, default_width, default_height, flags);
        //* 设置渲染的缩放质量为线性
        //* 用于设置 SDL 中的提示（Hint）。提示是一些可以影响库行为的设置，通常与性能、质量或兼容性相关。
        //* SDL_HINT_RENDER_SCALE_QUALITY 这是被设置的特定提示。它控制当 SDL 将纹理渲染为不同于其原始尺寸的大小时，所使用的缩放质量。该提示的可能取值包括：
            // "0" 或 "nearest"：最近邻采样（速度最快，但可能导致图像出现像素化或块状效果）。
            // "1" 或 "linear"：线性过滤（速度较慢，但会产生更平滑的结果）。
            // "2" 或 "best"：各向异性过滤（质量最高，但计算开销最大）。
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
        if (!window) {
            av_log(NULL, AV_LOG_FATAL, "Failed to create window: %s", SDL_GetError());
            do_exit(NULL);
        }

        //* vulkan渲染器设置
        if ( vk_renderer ) {
            //* 解析vukkan参数
            AVDictionary *dict = NULL;

            if (vulkan_params) {
                int ret = av_dict_parse_string(&dict, vulkan_params, "=", ":", 0);
                if (ret < 0) {
                    av_log(NULL, AV_LOG_FATAL, "Failed to parse, %s\n", vulkan_params);
                    do_exit(NULL);
                }
            }
            //* 创建vulkan渲染器
            ret = vk_renderer_create(vk_renderer, window, dict);
            av_dict_free(&dict);
            if (ret < 0) {
                av_log(NULL, AV_LOG_FATAL, "Failed to create vulkan renderer, %s\n", av_err2str(ret));
                do_exit(NULL);
            }
        } else {
            //* 没有vulkan渲染器，则由SDL根据系统创建一个使用硬件加速和垂直同步的渲染器
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
            if ( !renderer ) {
                //* 如果创建失败，则尝试创建无硬件加速的渲染器
                av_log(NULL, AV_LOG_WARNING, "Failed to initialize a hardware accelerated renderer: %s\n", SDL_GetError());
                renderer = SDL_CreateRenderer(window, -1, 0);
            }
            //* 获取渲染器信息，确保渲染器正确初始化
            if ( renderer ) {
                if (!SDL_GetRendererInfo(renderer, &renderer_info))
                    av_log(NULL, AV_LOG_VERBOSE, "Initialized %s renderer.\n", renderer_info.name);
            }
            //* 渲染器信息获取失败
            if ( !renderer || !renderer_info.num_texture_formats ) {
                av_log(NULL, AV_LOG_FATAL, "Failed to create window or renderer: %s", SDL_GetError());
                do_exit(NULL);
            }
        }
    }

    //* 打开视频流
    is = stream_open(input_filename, file_iformat);
    if (!is) {
        av_log(NULL, AV_LOG_FATAL, "Failed to initialize VideoState!\n");
        do_exit(NULL);
    }
    
    //* 开启事件循环处理播放逻辑、用户交互
    event_loop(is);

    /* never returns */

    return 0;
}
