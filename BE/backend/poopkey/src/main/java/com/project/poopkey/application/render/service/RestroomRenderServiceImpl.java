package com.project.poopkey.application.render.service;

import com.project.poopkey.application.render.dao.RestroomRenderDao;
import com.project.poopkey.application.render.dto.RestroomRender;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class RestroomRenderServiceImpl implements RestroomRenderService{

    @Autowired
    private RestroomRenderDao restroomRenderDao;

    @Override
    public RestroomRender findRestroomRender(int restroomId) {
        return restroomRenderDao.selectOne(restroomId);
    }

    @Override
    public List<RestroomRender> findRestroomRenderList(int buildingId, int floor) {
        return restroomRenderDao.selectList(buildingId, floor);
    }

    @Override
    public RestroomRender findRestroomRenderByStallId(long stallId) {
        int restroomId = restroomRenderDao.findRestroomId(stallId);
        return restroomRenderDao.selectOne(restroomId);
    }
}
