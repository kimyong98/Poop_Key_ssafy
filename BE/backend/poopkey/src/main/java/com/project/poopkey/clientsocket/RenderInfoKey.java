package com.project.poopkey.clientsocket;

public class RenderInfoKey {
    private int buildingId;
    private int restroomId;

    public RenderInfoKey(int buildingId, int restroomId) {
        this.buildingId = buildingId;
        this.restroomId = restroomId;
    }

    public int getBuildingId() {
        return buildingId;
    }

    public int getRestroomId() {
        return restroomId;
    }
}
//        예시 :
//        {
//            "buildingId": 2,
//            "restroomId": 4
//        }