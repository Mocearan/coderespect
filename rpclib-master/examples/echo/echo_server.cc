/*
 * @Author: Mass
 * @Date: 2022-08-30 14:18:43
 * @LastEditors: Mass
 * @LastEditTime: 2022-08-30 14:25:32
 * @FilePath: /coderespect/rpclib-master/examples/echo/echo_server.cc
 */
#include "rpc/server.h"
#include <iostream>

int main() {
    rpc::server srv(rpc::constants::DEFAULT_PORT);
    std::cout << "registered on  port " << srv.port() << std::endl;

    srv.bind("echo", [](std::string const& s) {
        return s;
    });

    srv.run();
    return 0;
}
