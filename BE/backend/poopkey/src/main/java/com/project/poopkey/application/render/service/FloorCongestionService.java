package com.project.poopkey.application.render.service;

import com.project.poopkey.application.render.dto.FloorCongestion;

import java.util.List;

public interface FloorCongestionService {
    List<FloorCongestion> findFloorCongestionList(int buildingid);
}
