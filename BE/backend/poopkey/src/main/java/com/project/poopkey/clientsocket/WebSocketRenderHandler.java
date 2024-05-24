package com.project.poopkey.clientsocket;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.project.poopkey.application.render.dto.FloorCongestion;
import com.project.poopkey.application.render.dto.RestroomRender;
import com.project.poopkey.application.render.service.FloorCongestionService;
import com.project.poopkey.application.render.service.RestroomRenderService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Component
@Slf4j
@ComponentScan(basePackages = "com.project.poopkey.application.render")
public class WebSocketRenderHandler extends TextWebSocketHandler {

    @Autowired
    private FloorCongestionService floorCongestionService;

    @Autowired
    private RestroomRenderService restroomRenderService;

    private final ObjectMapper objectMapper  = new ObjectMapper();

    List<WebSocketSession> sessions = new ArrayList<>();
    Map<WebSocketSession, RenderInfoKey> map = new HashMap<>();
    @Override
    public void afterConnectionEstablished(WebSocketSession session) throws Exception {
        System.out.println("connect");
        System.out.println("user 세션과 연결됨: "+session);
        sessions.add(session);
    }

    // 관리자한테 입력 받은 신호도 실시간 랜더링에 반영될 수 있도록 추가한다.
    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        String payload = message.getPayload();
        System.out.println(payload+": 이게 바로 보낸 메시지.");
        // 오브젝트 매퍼
        JsonNode jsonNode = objectMapper.readTree(payload);
        int buildingId = jsonNode.get("buildingId").asInt(); // json에서 해당 정보 추출
        int restroomId = jsonNode.get("restroomId").asInt();
        System.out.println("buildingId는:"+buildingId);
        System.out.println("restroomId는:"+restroomId);
        map.put(session, new RenderInfoKey(buildingId, restroomId));
        List<FloorCongestion> list = floorCongestionService.findFloorCongestionList(buildingId); // 화장실 혼잡 리스트 정보
        RestroomRender restroomRender = restroomRenderService.findRestroomRender(restroomId); // 화장실 내부 정보
        TextMessage renderInfo1 = new TextMessage(objectMapper.writeValueAsString(list));
        TextMessage renderInfo2 = new TextMessage(objectMapper.writeValueAsString(restroomRender));
        for(WebSocketSession ws : sessions){
            // 연결된 모든 세션에 정보 렌더링
            ws.sendMessage(renderInfo1);
            ws.sendMessage(renderInfo2);
        }
    }

    /**센서에서 입력 신호가 들어왔을 때 DB 트랜잭션이 끝나고 화면에 렌더링 되는 정보를 실시간으로 보내줌
     * 관리자가 바꾸는 입력도 포함된다.
     * */
    public void sendMsgToClient() throws IOException {
        System.out.println("이거는 센서쪽 신호가 클라이언트 쪽으로 전달되는 현상");
        for( WebSocketSession ws : sessions){
            int buildingId = map.get(ws).getBuildingId();
            int restroomId = map.get(ws).getRestroomId();
            List<FloorCongestion> list = floorCongestionService.findFloorCongestionList(buildingId);
            RestroomRender restroomRender = restroomRenderService.findRestroomRender(restroomId);
            TextMessage renderInfo1 = new TextMessage(objectMapper.writeValueAsString(list));
            TextMessage renderInfo2 = new TextMessage(objectMapper.writeValueAsString(restroomRender));
            ws.sendMessage(renderInfo1);
            ws.sendMessage(renderInfo2);
        }
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) throws Exception {
        map.remove(session); // buildingId와 restroomId 정보를 제거
        sessions.remove(session); // 세션정보 제거
        System.out.println("웹소켓 연결이 종료됨");
    }
}