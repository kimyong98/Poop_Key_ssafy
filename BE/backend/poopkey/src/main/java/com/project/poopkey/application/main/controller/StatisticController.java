package com.project.poopkey.application.main.controller;

import com.project.poopkey.application.main.dto.Statistic;
import com.project.poopkey.application.main.service.StatisticService;
import io.swagger.v3.oas.annotations.Operation;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@CrossOrigin("*")
@RequestMapping("/api")
public class StatisticController {
    @Autowired
    private StatisticService statisticService;
    // http://{SERVER_URL/api/master/statistics?buildingName={buildingName}&floor={floor}&gender={gender}
    @GetMapping("/master/statistics")
    @Operation(summary = "통계정보 확인(관리자)", description = "관리자 페이지에서 건물명, 층수, 성별을 입력하면 화장실 통계 정보를 반환하는 API")
    public ResponseEntity<?> statisticMasterFind(@RequestParam("buildingName")String buildingName, @RequestParam("floor")int floor, @RequestParam("gender")int gender){
        Statistic statistic = statisticService.findStatistic(buildingName, floor, gender);
        if(statistic==null)
            return new ResponseEntity<Void>(HttpStatus.NO_CONTENT);
        return new ResponseEntity<Statistic>(statistic, HttpStatus.OK);
    }
    // http://{SERVER_URL}/api/developer/openapi?buildingName={buildingName}&floor={floor}&gender={gender}
    @GetMapping("/developer/openapi")
    @Operation(summary = "통계정보 확인(개발자)", description = "관리자 통계와 똑같은 정보를 개발자를 위해 API형태로 제공함")
    public ResponseEntity<?> statisticDeveloperFind(@RequestParam("buildingName")String buildingName, @RequestParam("floor")int floor, @RequestParam("gender")int gender){
        Statistic statistic = statisticService.findStatistic(buildingName, floor, gender);
        if(statistic==null)
            return new ResponseEntity<Void>(HttpStatus.NO_CONTENT);
        return new ResponseEntity<Statistic>(statistic, HttpStatus.OK);
    }
}
