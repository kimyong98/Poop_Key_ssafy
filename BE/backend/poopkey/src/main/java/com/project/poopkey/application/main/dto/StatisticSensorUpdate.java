package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@Getter
@Setter
@ToString
public class StatisticSensorUpdate {
    private long stallId;
    private int usedNumber;
    private int tissueChangeNumber;
    private int breakNumber;
}
