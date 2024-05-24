package com.project.poopkey.application.render.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@Getter
@Setter
@ToString
public class StallRender {
//    dao 인덱싱 파라미터
    private long stallId;
    private int status;
    private String tissueStatus;
    private String content;
    private int row;
    private int col;
    private String detailURL; // 각 칸을 클릭하면 detail 페이지와 연결시킬 때 사용하는 URL
}
