package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

import java.sql.Timestamp;

@Getter
@Setter
@ToString
public class StallSensorUpdate {
    private long stallId;
    private int status;
    private Timestamp lastUsedTime;
    private int closed;
    private int usedNumberAfterCleaning;
    private int toiletStatus;
}
