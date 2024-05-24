package com.project.poopkey.application.main.controller;

import com.project.poopkey.application.main.dto.Building;
import com.project.poopkey.application.main.service.BuildingService;
import io.swagger.v3.oas.annotations.Operation;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.CrossOrigin;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@RestController
@CrossOrigin("*")
@RequestMapping("/api")
public class BuildingController {
    @Autowired
    private BuildingService buildingService;

    @GetMapping("/user/building/selectall")
    @Operation(summary = "전체 건물정보 반환", description = "DB상에 존재하는 모든 건물 정보를 반환, 네비게이션 연동 등에 활용")
    public ResponseEntity<?> buildingFindAll(){
        List<Building> list = buildingService.findAllBuilding();
        if(list==null || list.size()==0)
            return new ResponseEntity<Void>(HttpStatus.NO_CONTENT);
        return new ResponseEntity<List<Building>>(list, HttpStatus.OK);
    }
}
