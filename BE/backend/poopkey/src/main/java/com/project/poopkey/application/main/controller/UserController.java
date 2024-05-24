package com.project.poopkey.application.main.controller;

import com.project.poopkey.application.main.dto.User;
import com.project.poopkey.application.main.service.UserService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@CrossOrigin("*")
@RequestMapping("/api")
public class UserController {
    @Autowired
    private UserService userService;

    // http://{SERVER_URL}/api/user/regist
    @PostMapping("/user/regist")
    public ResponseEntity<?> userAdd(@RequestBody User user){
        userService.addUser(user);
        return new ResponseEntity<Void>(HttpStatus.CREATED);
    }


}
