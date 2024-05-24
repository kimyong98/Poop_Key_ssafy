package com.project.poopkey.application.main.controller;

import com.project.poopkey.application.main.dto.Report;
import com.project.poopkey.application.main.service.MasterService;
import com.project.poopkey.application.main.service.ReportService;
import com.project.poopkey.clientsocket.WebSocketReportHandler;
import io.swagger.v3.oas.annotations.Operation;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.io.IOException;
import java.util.List;

@RestController
@CrossOrigin("*")
@RequestMapping("/api")
public class ReportController {
    @Autowired
    private ReportService reportService;

    @Autowired
    private WebSocketReportHandler webSocketReportHandler;

    @PostMapping("/user/reports/regist")
    @Operation(summary = "신고내용 작성", description = "사용자가 신고를 작성하면 DB에 내용이 들어가고 " +
            "관리자에게 실시간으로 신고 리스트가 전달된다.")
    public ResponseEntity<Report> reportAdd(@RequestBody Report report) throws IOException {
        reportService.addReport(report);
        int masterId = reportService.findMasterId(report);
        System.out.println(masterId+"이게 찍히는 게 굉장히 중요");
        webSocketReportHandler.sendListToMaster(masterId);
        return new ResponseEntity<Report>(report, HttpStatus.CREATED);

    }

    /** 스웨거에서 테스트 하고 이후 WebSocket파일로 옮긴다. */
    @GetMapping("/master/reports/select")
    @Operation(summary = "관리자 신고 확인", description = "해당 관리자에게 들어온 모든 신고 리스트를 반환")
    public ResponseEntity<?> reportListFind(@RequestParam("masterId") int masterId){
        List<Report> list = reportService.findReportList(masterId);
        if(list==null || list.size()==0)
            return new ResponseEntity<Void>(HttpStatus.NO_CONTENT);
        return new ResponseEntity<List<Report>>(list, HttpStatus.OK);
    }

    /** 관리자가 신고글 읽었을 때 읽음 표시 */
    @PutMapping("/master/reports/update")
    @Operation(summary = "신고 읽음표시", description = "관리자가 신고글 읽었을 때 읽음으로 표시(checked:0->1) ")
    public ResponseEntity<Void> reportModify(@RequestParam("reportId") long reportId) throws IOException {
        reportService.modifyReport(reportId);
        int masterId = reportService.convertReportIdToMasterId(reportId);
        webSocketReportHandler.sendListToMaster(masterId);
        return new ResponseEntity<Void>(HttpStatus.OK);
    }
}
