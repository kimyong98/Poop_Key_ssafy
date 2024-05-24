package com.project.poopkey.application.main.dao;

import com.project.poopkey.application.main.dto.RestroomInfoInsert;
import com.project.poopkey.application.main.dto.Stall;

public interface StallDao {
    /** 화장실 칸 세부정보 표시(공통) */
    public Stall selectOne(long stallId);

}
