package com.project.poopkey.application.main.security.controller;

import com.project.poopkey.application.main.security.dao.MemberDao;
import com.project.poopkey.application.main.security.dto.SignRequest;
import com.project.poopkey.application.main.security.dto.SignResponse;
import com.project.poopkey.application.main.security.service.SignService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@RequiredArgsConstructor
public class SignController {
    private final MemberDao memberDao;
    private final SignService signService;

    @PostMapping("/login")
    public ResponseEntity<SignResponse> signin(@RequestBody SignRequest request) throws Exception{
        return new ResponseEntity<>(signService.login(request), HttpStatus.OK);
    }
    @PostMapping("/register")
    public ResponseEntity<Boolean> signup(@RequestBody SignRequest request) throws Exception{
//        System.out.println("registry 입력받음");
        return new ResponseEntity<>(signService.register(request), HttpStatus.OK);
    }
    @GetMapping("/user/get")
    public ResponseEntity<SignResponse> getUser(@RequestParam String email) throws Exception{
        return new ResponseEntity<>(signService.getMember(email), HttpStatus.OK);
    }
    @GetMapping("/admin/get")
    public ResponseEntity<SignResponse> getUserForAdmin(@RequestParam String email) throws Exception{
        return new ResponseEntity<>(signService.getMember(email), HttpStatus.OK);
    }
}
