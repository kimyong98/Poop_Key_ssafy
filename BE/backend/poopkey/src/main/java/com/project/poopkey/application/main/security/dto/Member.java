package com.project.poopkey.application.main.security.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.List;

@Getter
@Builder
@AllArgsConstructor
@NoArgsConstructor
public class Member {
    private int userId;
    private String password;
    private String email;
    private Timestamp createdDate;
    private String role; // "USER"

}
