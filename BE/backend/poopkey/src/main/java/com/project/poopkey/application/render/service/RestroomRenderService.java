package com.project.poopkey.application.render.service;

import com.project.poopkey.application.render.dto.RestroomRender;

import java.util.List;

public interface RestroomRenderService {
    RestroomRender findRestroomRender(int restroomId);
    List<RestroomRender> findRestroomRenderList(int buildingId, int floor);

    RestroomRender findRestroomRenderByStallId(long stallId);
}
