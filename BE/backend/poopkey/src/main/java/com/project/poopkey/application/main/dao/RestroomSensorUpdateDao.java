package com.project.poopkey.application.main.dao;

import org.apache.ibatis.annotations.Param;

public interface RestroomSensorUpdateDao {
    public void updateCongestion(@Param("restroomId") int restroomId, @Param("congestion") int congestion);
}