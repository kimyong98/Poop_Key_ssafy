package com.project.poopkey.application.main.dao;

import com.project.poopkey.application.main.dto.StallDetail;

public interface StallDetailDao {
    public StallDetail selectOne(long stallId);
}
