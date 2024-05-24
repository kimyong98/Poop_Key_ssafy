package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

import java.sql.Timestamp;

@Getter
@Setter
@ToString
public class Restroom {
    private int restroomId;
    private int buildingId;
    private int floor;
    private int congestion;
    private int gender;
    private int cleaning;
    private int height;
    private int width;
    private Timestamp lastCleaningTime;
}
