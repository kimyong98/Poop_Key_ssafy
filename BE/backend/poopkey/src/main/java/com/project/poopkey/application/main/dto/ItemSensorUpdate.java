package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@Getter
@Setter
@ToString
public class ItemSensorUpdate {
    private long itemId;
    private int tissue;
    private long stallId;
}
