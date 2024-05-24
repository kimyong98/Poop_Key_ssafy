package com.project.poopkey.application.main.controller;

import com.project.poopkey.application.main.dto.StallDetail;
import com.project.poopkey.application.main.service.StallDetailService;
import io.swagger.v3.oas.annotations.Operation;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@CrossOrigin("*")
@RequestMapping("/api")
public class StallDetailController {
    @Autowired
    private StallDetailService stallDetailService;
    // http://{SERVER_URL}/api/master/stalls/detail?stallId={stallId}
    @GetMapping("/master/stalls/detail")
    @Operation(summary = "화장실 세부정보 제공(관리자)" ,description = "최근 사용시간과 청소하고 사용한 횟수를 반환")
    public ResponseEntity<?> stallMasterDetailFind(@RequestParam("stallId")long stallId){
        StallDetail stallDetail = stallDetailService.findStallDetail(stallId);
        if(stallDetail==null)
            return new ResponseEntity<Void>(HttpStatus.NO_CONTENT);
        return new ResponseEntity<StallDetail>(stallDetail, HttpStatus.OK);
    }

    @GetMapping("/user/stalls/detail")
    @Operation(summary = "화장실 세부정보 제공(사용자)" ,description = "최근 사용시간과 청소하고 사용한 횟수를 반환")
    public ResponseEntity<?> stallUserDetailFind(@RequestParam("stallId")long stallId){
        StallDetail stallDetail = stallDetailService.findStallDetail(stallId);
        if(stallDetail==null)
            return new ResponseEntity<Void>(HttpStatus.NO_CONTENT);
        return new ResponseEntity<StallDetail>(stallDetail, HttpStatus.OK);
    }
}
