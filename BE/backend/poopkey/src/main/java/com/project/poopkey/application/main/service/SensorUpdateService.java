package com.project.poopkey.application.main.service;

public interface SensorUpdateService {

    void modifyTissue(long stallId, int tissue);
    void modifyOccupied(long stallId);
    void modifyVacant(long stallId);
    void modifyBreak(long stallId);
    void modifyCongestion(int restroomId, int congestion);
}
