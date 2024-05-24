package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.StallUpdateDao;
import com.project.poopkey.application.main.dto.StallUpdate;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class StallUpdateServiceImpl implements StallUpdateService{
    @Autowired
    private StallUpdateDao stallUpdateDao;

    @Override
    public void modifyStallUpdate(StallUpdate stallUpdate) {
        stallUpdateDao.update(stallUpdate);
    }
}
