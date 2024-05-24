package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

import java.sql.Timestamp;

@Getter
@Setter
@ToString
public class Master {
    private int masterId;
    private String password;
    private String name;
    private String phone;
    private String deviceId;
    private Timestamp createdDate;
    private String email;
    private byte[] profile;
    private String address;
    private Timestamp deletedDate;
}
