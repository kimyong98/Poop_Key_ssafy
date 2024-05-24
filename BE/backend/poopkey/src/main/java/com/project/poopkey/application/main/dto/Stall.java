package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

import java.sql.Timestamp;

@Getter
@Setter
@ToString
public class Stall {
    private long stallId;
    private Timestamp lastUsedTime;
    private int usedNumberAfterCleaning;
}
