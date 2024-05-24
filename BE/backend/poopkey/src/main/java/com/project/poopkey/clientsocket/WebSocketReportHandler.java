/**
 * 사용자로부터 들어온 신고를
 * 관리자에게 웹소켓으로 실시간으로 알려준다.
 * */
package com.project.poopkey.clientsocket;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.project.poopkey.application.main.dto.Report;
import com.project.poopkey.application.main.service.ReportService;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
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
public class WebSocketReportHandler extends TextWebSocketHandler {

    // 일반적인 HttpMapping 방식으로 메시지를 전달받으면 웹소켓으로 알림을 전송하면 됨.
    // 이후 jwt 토큰 고유값에 대한 연동 작업도 필요.

    private final ObjectMapper objectMapper = new ObjectMapper();

    private List<WebSocketSession> sessions = new ArrayList<>();

    /** masterId를 받으면 해당 소켓세션을 호출 */
    private Map<Integer, WebSocketSession> masterIdToSession = new HashMap<>();
    /** 세션 해제시 자원 반납을 위해 반대 함수도 필요*/
    private Map<WebSocketSession, Integer> sessionToMasterId = new HashMap<>();

    @Autowired
    private ReportService reportService;

    @Override
    public void afterConnectionEstablished(WebSocketSession session) throws Exception {
        System.out.println("report 세션 연결+"+session);
        sessions.add(session);
    }

    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) throws Exception {
        // masterId 정보를 관리자쪽에서 백엔드 서버로 보낸다.
        // 이 과정은 이후 jwt 토큰 방식으로 대체된다.
        // 연결이 진행되고 처음 masterId 가 전달되면 자신에게 등록된 메시지를 받는다.
        // 안 읽음 상태의 메시지가 1개라도 존재하면 message send로 알려준다.

        // 클라이언트가 보내는 경우는 ReportHandler단에서 처리가 이루어진다.
        // ReportHandler는 WebSocketReportHandler를 이용해서 실시간 처리를 한다.


        String payload = message.getPayload();
        System.out.println(payload+": 이게 바로 보낸 메시지.");
        // 오브젝트 매퍼
        JsonNode jsonNode = objectMapper.readTree(payload);
        int masterId = jsonNode.get("masterId").asInt(); // json에서 해당 정보 추출
//        {
//            "masterId": 4
//        }
        masterIdToSession.put(masterId, session);
        sessionToMasterId.put(session, masterId);
        List<Report> reportList = reportService.findReportList(masterId);
        TextMessage reportInfo = new TextMessage(objectMapper.writeValueAsString(reportList));
        session.sendMessage(reportInfo);
    }

    public void sendListToMaster(int masterId) throws IOException {
        List<Report> reportList = reportService.findReportList(masterId);
        WebSocketSession ws = masterIdToSession.get(masterId);
        TextMessage reportInfo = new TextMessage(objectMapper.writeValueAsString(reportList));
        ws.sendMessage(reportInfo);
    }


    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) throws Exception {
        int masterId = sessionToMasterId.get(session);
        // map의 자원 반납 절차
        masterIdToSession.remove(masterId);
        sessionToMasterId.remove(session);

        System.out.println("report 세션 해제+"+session);
        sessions.remove(session);
    }
}
