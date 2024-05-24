package com.project.poopkey.application.main.dao;

import org.apache.ibatis.annotations.Param;

public interface StatisticSensorUpdateDao {
    public void updateTissue(@Param("stallId") long stallId, @Param("tissue") int tissue);
    public void updateOccupied(long stallId);
    public void updateBreak(long stallId);
}
