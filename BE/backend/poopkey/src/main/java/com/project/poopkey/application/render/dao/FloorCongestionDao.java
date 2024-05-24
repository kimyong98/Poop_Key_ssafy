package com.project.poopkey.application.render.dao;

import com.project.poopkey.application.render.dto.FloorCongestion;

import java.util.List;

public interface FloorCongestionDao {
    public List<FloorCongestion> selectList(int buildingId);
}
