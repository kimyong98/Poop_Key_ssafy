package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@Getter
@Setter
@ToString
public class Building {
    private int buildingId;
    private int masterId;
    private String buildingName;
    private double buildingLat;
    private double buildingLng;
    private int congestion;
    private String address;
}
