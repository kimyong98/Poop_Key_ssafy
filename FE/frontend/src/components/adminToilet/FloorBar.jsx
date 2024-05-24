// FloorBar.js
import styles from "./FloorBar.module.css";
import React, { useEffect, useState, useRef } from "react";
import { getCongestion, Web_Socket_URL } from "../../util/API";
import CongestionItem from "./CongestionItem";

const FloorBar = ({
  selectFloor,
  restroomData,
  handleFloorClick,
  updataWebSocket,
}) => {
  const [congestionData, setCongestionData] = useState();
  const [isUpdate, setIsUpdate] = useState();
  const webSocket = useRef(null);
  let floorFilter = 1;

  useEffect(() => {
    if (restroomData !== undefined) {
      const buildingId = restroomData[0].buildingId;
      const getCongestionData = async () => {
        try {
          const response = await getCongestion(buildingId);
          setCongestionData(response);
        } catch (e) {
          console.log(e);
        }
      };
      getCongestionData();
    }
  }, [restroomData, isUpdate]);

  //useEffert 사용하면서 데이터를 받아오고 setCongetstion으로 초기화 해주기
  //congestionData랑 데이터가 같은지 확인하기!!! (매우중요)

  // WebSocket
  useEffect(() => {
    webSocket.current = new WebSocket(Web_Socket_URL);

    webSocket.current.onopen = () => {
      //console.log("웹소켓 연결 성공!!");
      const data = {
        buildingId: 1,
        restroomId: 1,
        masterId: 1, //로컬 스토리지에서 가져온다. or useParams 사용할것
      };

      const jsonData = JSON.stringify(data);

      sendMessage(jsonData);
    };

    webSocket.current.onerror = (error) => {
      //console.log("웹소켓 연결 에러:", error);
    };

    webSocket.current.onmessage = (event) => {
      //console.log("웹소켓 메시지 수신:");
      updataWebSocket(Math.random());
      setIsUpdate(Math.random());
    };

    webSocket.current.onclose = (event) => {
      //console.log("웹소켓 연결 종료:", event);
    };

    return () => {
      if (webSocket.current.readyState === WebSocket.OPEN) {
        webSocket.current?.close();
      }
    };
  }, [selectFloor]);

  // 메시지 보내기
  const sendMessage = (message) => {
    if (webSocket.current.readyState === WebSocket.OPEN) {
      webSocket.current.send(message);
    }
  };

  return (
    <div className={styles.floorBar}>
      <div className={styles.floorList}>
        {restroomData &&
          restroomData.map((restroom) => {
            // restroom.floor와 일치하는 화장실 혼잡 데이터 추출
            const currentFloorCongestion =
              congestionData &&
              congestionData.filter((data) => data.floor == restroom.floor);
            if (
              restroom.floor >= floorFilter &&
              restroom.restroomId !== undefined
            ) {
              if (restroom.floor == floorFilter) {
                floorFilter++;
              } else if (restroom.floor > floorFilter) {
                floorFilter = restroom.floor + 1;
              }

              const list =
                currentFloorCongestion &&
                currentFloorCongestion[0] &&
                currentFloorCongestion[0].list;
              let congestionMen = -1;
              let congestionWomen = -1;

              if (list && list.length > 0) {
                if (list.length == 1) {
                  list[0].gender == 0
                    ? (congestionMen = list[0].congestion)
                    : (congestionWomen = list[0].congestion);
                } else if (list.length == 2) {
                  congestionMen = list[0].congestion;
                  congestionWomen = list[1].congestion;
                }
              } else {
                return;
              }

              return (
                <div
                  key={restroom.restroomId}
                  className={
                    selectFloor == restroom.floor
                      ? styles.floorItem_on
                      : styles.floorItem_off
                  }
                  onClick={() => handleFloorClick(restroom.floor)}
                >
                  <div className={styles.floorCongestion}>
                    {congestionMen != -1 ? (
                      <CongestionItem status={congestionMen}></CongestionItem>
                    ) : (
                      <CongestionItem status={-1}></CongestionItem>
                    )}

                    {congestionWomen != -1 ? (
                      <CongestionItem status={congestionWomen}></CongestionItem>
                    ) : (
                      <CongestionItem status={-1}></CongestionItem>
                    )}
                  </div>
                  <p className={styles.floorText}>{restroom.floor}F</p>
                </div>
              );
            }
            return null;
          })}
      </div>
    </div>
  );
};

export default FloorBar;
