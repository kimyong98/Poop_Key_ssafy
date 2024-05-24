package com.project.poopkey.application.main.service;

import com.project.poopkey.application.main.dto.Restroom;
import com.project.poopkey.application.main.dto.RestroomInfoInsert;

import java.util.List;

public interface RestroomService {
    List<Restroom> findRestroomList(int buildingId);

    void addRestroom(RestroomInfoInsert restroomInfoInsert);

    void modifyRestroomClean(int restroomId);
}
