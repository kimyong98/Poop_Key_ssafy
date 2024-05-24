package com.project.poopkey.application.main.controller;

import com.project.poopkey.application.main.dto.Stall;
import com.project.poopkey.application.main.service.StallService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@CrossOrigin("*")
@RequestMapping("/api")
public class StallController {
    @Autowired
    private StallService stallService;

}
