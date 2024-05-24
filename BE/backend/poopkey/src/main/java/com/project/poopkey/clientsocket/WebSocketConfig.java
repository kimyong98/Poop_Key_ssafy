package com.project.poopkey.clientsocket;


import lombok.RequiredArgsConstructor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.Configuration;
import org.springframework.messaging.simp.config.MessageBrokerRegistry;
import org.springframework.web.socket.config.annotation.*;

@Configuration
@EnableWebSocket
@RequiredArgsConstructor
public class WebSocketConfig implements WebSocketConfigurer {

    private final WebSocketRenderHandler webSocketRenderHandler;

    private final WebSocketReportHandler webSocketReportHandler;
    @Override
    public void registerWebSocketHandlers(WebSocketHandlerRegistry registry) {
        registry.addHandler(webSocketRenderHandler, "/ws").setAllowedOrigins("*").setAllowedOriginPatterns("/ws")
                .addHandler(webSocketReportHandler, "/ws/report").setAllowedOrigins("*").setAllowedOriginPatterns("/ws/report");
    }

}
