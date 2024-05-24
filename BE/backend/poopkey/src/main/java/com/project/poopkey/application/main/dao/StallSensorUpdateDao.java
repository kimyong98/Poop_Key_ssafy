package com.project.poopkey.application.main.dao;

public interface StallSensorUpdateDao {
    public void updateOccupied(long stallId);
    public void updateVacant(long stallId);
    public void updateBreak(long stallId);
}
