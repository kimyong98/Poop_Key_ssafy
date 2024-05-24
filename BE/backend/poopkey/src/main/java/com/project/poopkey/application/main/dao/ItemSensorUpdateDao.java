package com.project.poopkey.application.main.dao;

import org.apache.ibatis.annotations.Param;

public interface ItemSensorUpdateDao {
    public void update(@Param("stallId") long stallId, @Param("tissue") int tissue);
}
