package com.project.poopkey.application.main.controller;

import com.project.poopkey.application.main.service.MasterService;
import io.swagger.v3.oas.annotations.Operation;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@CrossOrigin("*")
@RequestMapping("/api")
public class MasterController {
    @Autowired
    private MasterService masterService;
    @PutMapping("/master/delete")
    @Operation(summary = "관리자 계정탈퇴", description = "관리자 계정의 모든 column 정보를 지우고 탈퇴일을 기록")
    public ResponseEntity<?> masterRemove(@RequestParam("email")String email){
        masterService.removeMaster(email);
        return new ResponseEntity<Void>(HttpStatus.OK);
    }
}
