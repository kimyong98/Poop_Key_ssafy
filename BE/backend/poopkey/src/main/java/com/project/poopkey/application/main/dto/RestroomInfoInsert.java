/**
 * Master로부터 입력받은 건물 정보를 등록할 때 사용하는 DTO
 * */
package com.project.poopkey.application.main.dto;

import lombok.Getter;
import lombok.Setter;
import lombok.ToString;

import java.util.List;

@Getter
@Setter
@ToString
public class RestroomInfoInsert {
    // 아래 두 줄은 마이바티스 insert 인덱스 계산에 사용
    private int index; // restroom_id
    private long stallIndex;

    private int buildingId;
    private int floor;
    private int gender;
    private int height;
    private int width;
    List<StallInfoInsert> list;
}
