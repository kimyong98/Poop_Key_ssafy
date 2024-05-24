package com.project.poopkey.application.main.controller;

import com.project.poopkey.application.main.dto.Restroom;
import com.project.poopkey.application.main.dto.RestroomInfoInsert;
import com.project.poopkey.application.main.service.RestroomService;
import io.swagger.v3.oas.annotations.Operation;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@CrossOrigin("*")
@RequestMapping("/api")
public class RestroomController {
    // 기능에 따라 2개 이상 service객체를 가져와야 될 수 있다.
    // 화장실을 청소하는 경우 다른 table에 대한 정보를 같이 update할 필요가 있다.

    @Autowired
    private RestroomService restroomService;

    @GetMapping("/user/restroom/select")
    @Operation(summary = "화장실 정보 반환(사용자)", description = "건물id를 입력하면 그 건물 내 모든 화장실 정보를 반환")
    public ResponseEntity<?> restroomUserFindList(@RequestParam("buildingId") int buildingId){
        List<Restroom> list = restroomService.findRestroomList(buildingId);
        if(list==null || list.size()==0)
            return new ResponseEntity<Void>(HttpStatus.NO_CONTENT);
        return new ResponseEntity<List<Restroom>>(list, HttpStatus.OK);
    }

    @GetMapping("/master/restroom/select")
    @Operation(summary = "화장실 정보 반환(관리자)", description = "건물id를 입력하면 그 건물 내 모든 화장실 정보를 반환")
    public ResponseEntity<?> restroomMasterFindList(@RequestParam("buildingId") int buildingId){
        List<Restroom> list = restroomService.findRestroomList(buildingId);
        if(list==null || list.size()==0)
            return new ResponseEntity<Void>(HttpStatus.NO_CONTENT);
        return new ResponseEntity<List<Restroom>>(list, HttpStatus.OK);
    }

    @PostMapping("/master/restroom/insert")
    @Operation(summary = "화장실 신규 등록", description = "관리자가 새로운 화장실에 대한 정보를 DB에 등록하는 API")
    public ResponseEntity<Void> restroomAdd(@RequestBody RestroomInfoInsert restroomInfoInsert){
        restroomService.addRestroom(restroomInfoInsert);
        return new ResponseEntity<Void>(HttpStatus.CREATED);
    }

    @PutMapping("/master/restroom/clean")
    @Operation(summary = "화장실 청소 상태 변경", description = "관리자가 현재 화장실 청소상태를 변경(0->1 & 1->0)")
    public ResponseEntity<Void> restroomModifyClean(@RequestParam int restroomId){
        restroomService.modifyRestroomClean(restroomId);
        return new ResponseEntity<Void>(HttpStatus.OK);
    }

}
