package com.project.poopkey.application.render.controller;

import com.project.poopkey.application.render.dto.FloorCongestion;
import com.project.poopkey.application.render.dto.RestroomRender;
import com.project.poopkey.application.render.service.FloorCongestionService;
import com.project.poopkey.application.render.service.RestroomRenderService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@CrossOrigin("*")
@RequestMapping("/api")
public class RenderTestTempController {
    @Autowired
    private RestroomRenderService restroomRenderService;

    @Autowired
    private FloorCongestionService floorCongestionService;

    @GetMapping("/api/tmptest1")
    private ResponseEntity<RestroomRender> test1(@RequestParam("restroomId")int restroomId){
        RestroomRender a = restroomRenderService.findRestroomRender(restroomId);
        return new ResponseEntity<RestroomRender>(a, HttpStatus.OK);
    }
    @GetMapping("/api/tmptest2")
    private ResponseEntity<List<FloorCongestion>> test2(@RequestParam("buildingId")int buildingId){
        List<FloorCongestion> list = floorCongestionService.findFloorCongestionList(buildingId);
        return new ResponseEntity<List<FloorCongestion>>(list, HttpStatus.OK);
    }

    @GetMapping("/api/tmptest3")
    private ResponseEntity<List<RestroomRender>> test3(@RequestParam("buildingId")int buildingId, @RequestParam("floor")int floor){
        List<RestroomRender> list = restroomRenderService.findRestroomRenderList(buildingId, floor);
        return new ResponseEntity<List<RestroomRender>>(list, HttpStatus.OK);
    }

    @GetMapping("/api/tmptest4")
    private ResponseEntity<RestroomRender> test4(@RequestParam("stallId")long stallId){
        RestroomRender restroomRender = restroomRenderService.findRestroomRenderByStallId(stallId);
        return new ResponseEntity<RestroomRender>(restroomRender, HttpStatus.OK);
    }
}
