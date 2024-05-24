package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

import java.sql.Timestamp;

@Getter
@Setter
@ToString
public class StallStatistic {
    private int usedNumber;
    private Timestamp lastCleanDate;
    private int tissueChangeNumber;
    private int breakNumber;
}
