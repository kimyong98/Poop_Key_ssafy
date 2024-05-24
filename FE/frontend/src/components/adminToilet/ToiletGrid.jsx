import styles from "./ToiletGrid.module.css";
import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { getStall } from "../../util/API";

const ToiletGrid = ({ restroom }) => {
  const [stallData, setStallData] = useState(); //화장실정보 + 화장실 칸 list
  const [rows, setRows] = useState(0); // 화장실의 row
  const [cols, setCols] = useState(0); // 화장실의 col
  const [floor, setFloor] = useState(0);
  const navigate = useNavigate();

  //status: 사용가능:0 사용중:1 고장:2
  //content: 빈칸:0 화장실:1 입구:2
  useEffect(() => {
    if (restroom !== undefined) {
      const getStallData = async () => {
        try {
          const response = await getStall(restroom[0].restroomId);
          setStallData(response);
          setRows(response.height); // 화장실 행
          setCols(response.width); // 열
          setFloor(response.floor); // 층
        } catch (e) {
          console.log(e);
        }
      };
      getStallData();
    }
  }, [restroom]);

  //빈칸
  const itemStyle = {
    boxSizing: "border-box",
    width: `${Math.min(90 / rows, 90 / cols)}%`,
    height: `${Math.min(90 / rows, 90 / cols)}%`,
    visibility: "hidden",
  };
  const tissue = { src: process.env.PUBLIC_URL + `/assets/tissue.png` };

  // 입구 + img
  const enterStyle = {
    boxSizing: "border-box",
    width: `${Math.min(90 / rows, 90 / cols)}%`,
    height: `${Math.min(90 / rows, 90 / cols)}%`,
    //border: "3px solid white",
    backgroundColor: "skyblue",
    borderRadius: "20px",
  };
  const enterImg = { src: process.env.PUBLIC_URL + `/assets/enter.png` };

  // 화장실칸 사용가능 style + img
  const toiletItemStyle0 = {
    boxSizing: "border-box",
    width: `${Math.min(90 / rows, 90 / cols)}%`,
    height: `${Math.min(90 / rows, 90 / cols)}%`,
    //border: "3px solid white",
    backgroundColor: "#90ee90",
    borderRadius: "20px",
    cursor: "pointer",
  };
  const img0 = { src: process.env.PUBLIC_URL + `/assets/toilet.png` };

  // 화장실칸 사용중 style + img
  const toiletItemStyle1 = {
    boxSizing: "border-box",
    width: `${Math.min(90 / rows, 90 / cols)}%`,
    height: `${Math.min(90 / rows, 90 / cols)}%`,
    //border: "3px solid white",
    backgroundColor: "#FF6E6E",
    borderRadius: "20px",
    cursor: "pointer",
  };
  const img1 = { src: process.env.PUBLIC_URL + `/assets/man.png` };

  // 화장실칸 고장 style + img
  const toiletItemStyle2 = {
    boxSizing: "border-box",
    width: `${Math.min(90 / rows, 90 / cols)}%`,
    height: `${Math.min(90 / rows, 90 / cols)}%`,
    //border: "3px solid white",
    backgroundColor: "#FFFA82",
    borderRadius: "25px",
    cursor: "pointer",
  };
  const img2 = { src: process.env.PUBLIC_URL + `/assets/warning.png` };

  // 화장실칸 점검중 style + img
  const toiletItemStyle3 = {
    boxSizing: "border-box",
    width: `${Math.min(90 / rows, 90 / cols)}%`,
    height: `${Math.min(90 / rows, 90 / cols)}%`,
    //border: "3px solid white",
    backgroundColor: "#C0C0C0",
    borderRadius: "25px",
    cursor: "pointer",
  };
  const img3 = { src: process.env.PUBLIC_URL + `/assets/repair.png` };

  // 화장실칸 id만 넘겨줘서 필요한 데이터는 거기 가서 받던지, 아님 여기서 객체 자체를 넘겨줘도 됨!
  const handleDetailClick = (data) => {
    // 관리자 상태변경
    navigate(
      `/admin/toiletDetail/${restroom[0].buildingId}/${floor}/${data.stallId}`
    );
  };

  // 화장실 칸의 좌표를 받아, 격자판에 화장실 칸 위치 표시

  return (
    <div className={styles.gridContainer}>
      {stallData &&
        stallData.list.map((item) => {
          if (item.content == 0) {
            //빈칸
            return (
              <div
                key={item.stallId}
                style={itemStyle}
                row={item.row}
                col={item.col}
              ></div>
            );
          } else if (item.content == 2) {
            //입구
            return (
              <div
                key={item.stallId}
                className={styles.iconContainer}
                style={enterStyle}
                row={item.row}
                col={item.col}
              >
                <img className={styles.icon} src={enterImg.src} />
              </div>
            );
          } else if (item.status === 0) {
            //사용가능
            return (
              <div
                key={item.stallId}
                className={styles.iconContainer}
                style={toiletItemStyle0}
                row={item.row}
                col={item.col}
                onClick={() => handleDetailClick(item)}
              >
                {item.tissueStatus.length > 0 ? (
                  <img className={styles.icon} src={tissue.src} />
                ) : (
                  <img className={styles.icon} src={img0.src} />
                )}
              </div>
            );
          } else if (item.status === 1) {
            //사용중
            return (
              <div
                key={item.stallId}
                className={styles.iconContainer}
                style={toiletItemStyle1}
                row={item.row}
                col={item.col}
                onClick={() => handleDetailClick(item)}
              >
                {item.tissueStatus.length > 0 ? (
                  <img className={styles.icon} src={tissue.src} />
                ) : (
                  <img className={styles.icon} src={img1.src} />
                )}
              </div>
            );
          } else if (item.status === 2) {
            //고장
            return (
              <div
                key={item.stallId}
                className={styles.iconContainer}
                style={toiletItemStyle2}
                row={item.row}
                col={item.col}
                onClick={() => handleDetailClick(item)}
              >
                <img className={styles.icon} src={img2.src} />
              </div>
            );
          } else if (item.status === 3) {
            //점검중
            return (
              <div
                key={item.stallId}
                className={styles.iconContainer}
                style={toiletItemStyle3}
                row={item.row}
                col={item.col}
                onClick={() => handleDetailClick(item)}
              >
                <img className={styles.icon} src={img3.src} />
              </div>
            );
          }
        })}
    </div>
  );
};
export default ToiletGrid;
