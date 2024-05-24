/**
 * Master로부터 입력받은 건물 정보를 등록할 때 사용하는 DTO,
 * RestroomInfoInsert에 내장된 DTO
 * */
package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

@Getter
@Setter
@ToString
public class StallInfoInsert {
    private long stallId;
    private int content;
    private int row;
    private int col;
}
