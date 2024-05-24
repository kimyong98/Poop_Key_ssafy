package com.project.poopkey.netty;

import com.project.poopkey.application.main.service.SensorUpdateService;
import com.project.poopkey.clientsocket.WebSocketRenderHandler;
import io.netty.buffer.ByteBuf;
import io.netty.channel.ChannelHandler;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelInboundHandlerAdapter;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.stereotype.Component;

import java.nio.charset.Charset;

@ComponentScan(basePackages = {"com.project.poopkey.application.main", "com.project.poopkey.clientsocket"})
@Component
@ChannelHandler.Sharable
public class NettySocketServerHandler extends ChannelInboundHandlerAdapter {

    @Autowired
    private SensorUpdateService sensorUpdateService;

    @Autowired
    private WebSocketRenderHandler webSocketRenderHandler;

    @Override
    public void channelRead(ChannelHandlerContext ctx, Object msg) throws Exception {
        String readMsg = ((ByteBuf)msg).toString(Charset.forName("UTF-8"));
        ctx.write(msg);
        System.out.println("message from received: "+readMsg);

        String[] parser = readMsg.split(",");
        System.out.println(parser[0]);
        System.out.println(parser[1]);
        System.out.println(parser[2]);
        long stallId = 0;
        // DB closed 됐을 때 신호 감지하는 것만 신경쓰기.
        switch (parser[0]){
            case "tissueStatus":
                int tissue = Integer.parseInt(parser[1]);
                stallId = Long.parseLong(parser[2]);
                sensorUpdateService.modifyTissue(stallId, tissue);
                break;
            case "toiletOccupied":
                // 그렇지만 DB업데이트 시간차가 조금 나는 건 확인할 필요가 있다.
                stallId = Long.parseLong(parser[1]);
                sensorUpdateService.modifyOccupied(stallId);
                break;
            case "toiletVacant":
                stallId = Long.parseLong(parser[1]);
                sensorUpdateService.modifyVacant(stallId);
                break;
            case "toiletBreak":
                stallId = Long.parseLong(parser[1]);
                sensorUpdateService.modifyBreak(stallId);
                break;
            case "congestion":
                int congestion = Integer.parseInt(parser[1]);
                int restroomId = Integer.parseInt(parser[2]);
                sensorUpdateService.modifyCongestion(restroomId, congestion);
                break;

        }
        webSocketRenderHandler.sendMsgToClient(); // 업데이트 된 정보를 클라이언트에 랜더링
    }

    @Override
    public void exceptionCaught(ChannelHandlerContext ctx, Throwable cause) throws Exception {
        cause.printStackTrace();
        ctx.close();
    }
}

//센서 tcp 통신 문자열 포맷 정의해두기
// "tissueStatus,100,4,"
// "toiletOccupied,1,-1," -- 화장실 PK를 센서에서 가지고 있음. 사전에 부여받는 형태.
// "toiletVacant,2,-1," -- PK
// "toiletBreak,4,-1," -- PK
// "congestion,1,3," -- 0, 1, 2로 혼잡도 표시, PK
// 대표적인 테스트코드 tissueStatus,1100,1,
