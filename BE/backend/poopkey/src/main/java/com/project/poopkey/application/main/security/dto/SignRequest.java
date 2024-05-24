package com.project.poopkey.application.main.security.dto;

import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class SignRequest {
    private String email;
    private String password;
}
