package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

import java.util.List;

@Getter
@Setter
@ToString
public class Statistic {
    private String buildingName;
    private int floor;
    private int gender;
    private List<StallStatistic> list;
}
