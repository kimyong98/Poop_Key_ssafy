package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dto.Stall;

public interface StallService {
    /** 화장실 칸 세부정보 표시(공통) */
    public Stall findStall(long stallId);

}
