package com.project.poopkey.application.render.dao;

import com.project.poopkey.application.render.dto.RestroomRender;

import java.util.List;

public interface RestroomRenderDao {
    public RestroomRender selectOne(int restroomId);
    public List<RestroomRender> selectList(int buildingId, int floor);
    public int findRestroomId(long stallId);
}
