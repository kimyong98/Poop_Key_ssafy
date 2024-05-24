package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dao.ItemSensorUpdateDao;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class ItemServiceImpl implements ItemService {

    @Autowired
    private ItemSensorUpdateDao itemDao;

    @Override
    public void modifyItem(long stallId, int tissue) {
        itemDao.update(stallId, tissue);
    }
}
