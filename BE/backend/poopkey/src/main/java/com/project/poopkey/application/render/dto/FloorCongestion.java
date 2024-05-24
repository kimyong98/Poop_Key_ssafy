package com.project.poopkey.application.render.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

import java.util.List;

@Getter
@Setter
@ToString
public class FloorCongestion {
    private int floor;
    private String URL;
    private List<Congestion> list;
}
