package com.project.poopkey.application.main.dao;

import com.project.poopkey.application.main.dto.Restroom;
import com.project.poopkey.application.main.dto.RestroomInfoInsert;

import java.util.List;

public interface RestroomDao {
    public List<Restroom> selectList(int buildingId);

    public void insertOne(RestroomInfoInsert restroomInfoInsert);

    public  void updateRestroomClean(int restroomId);
}
