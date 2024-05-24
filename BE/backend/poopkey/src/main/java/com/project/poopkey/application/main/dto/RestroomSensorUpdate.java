package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@Getter
@Setter
@ToString
public class RestroomSensorUpdate {
    private int restroomId;
    private int congestion;
}
