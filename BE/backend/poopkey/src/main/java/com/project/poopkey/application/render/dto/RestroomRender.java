package com.project.poopkey.application.render.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

import java.util.List;

@Getter
@Setter
@ToString
public class RestroomRender {
    private int restroomId;
    private int gender;
    private int floor;
    private int width;
    private int height;
    List<StallRender> list;
}
