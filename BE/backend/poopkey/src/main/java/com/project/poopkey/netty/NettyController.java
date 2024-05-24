package com.project.poopkey.netty;

import jakarta.annotation.PostConstruct;
import jakarta.annotation.PreDestroy;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Controller;

@Controller
public class NettyController {

    @Autowired
    private NettySocketServer nettySocketServer;

    @PostConstruct
    private void start(){
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
//                    new NettySocketServer(5010).run();
                    nettySocketServer.run();
                } catch (Exception e){
                    e.printStackTrace();
                }
            }
        }).start();
    }
    @PreDestroy
    private void destroy(){

    }
}
