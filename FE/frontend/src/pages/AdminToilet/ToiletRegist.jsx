import styles from "./ToiletRegist.module.css";
import { useParams, useNavigate } from "react-router-dom";
import { useEffect, useState } from "react";
import RegistItem from "../../components/adminToilet/RegistItem";
import { createRestroom } from "../../util/AdminAPI";
import Swal from "sweetalert2";

const ToiletRegist = () => {
  const navigate = useNavigate();
  const { buildingId } = useParams();
  const [stallCount, setStallCount] = useState(0);
  const [gender, setGender] = useState(0);
  const [floor, setFloor] = useState(1);
  const [stallData, setStallData] = useState([]);
  const [gridSize, setgridSize] = useState({
    row: 3,
    col: 3,
  });

  const numberOptions = Array.from({ length: 20 }, (_, index) => index + 1);

  // 행과 열의 변경을 감지하고 둘다 숫자가 들어오면 화장실칸 생성
  useEffect(() => {
    setStallCount(0);
    if (Number(gridSize.col) && Number(gridSize.col)) {
      createGrid(gridSize.row, gridSize.col);
    }
  }, [gridSize]);

  // 입력된 행과열을 감지하여 상태 변경
  const handleInputChange = (e) => {
    const { name, value } = e.target;
    if (value > 5) {
      setgridSize({
        ...gridSize,
        [name]: 5,
      });
    } else {
      setgridSize({
        ...gridSize,
        [name]: value,
      });
    }
  };

  // 선택된 성별을 감지하여 상태 변경
  const handleGenderChange = (e) => {
    const { value } = e.target;
    setGender(value);
  };

  // 선택된 층을 감지하여 상태 변경
  const handleFloorChange = (e) => {
    const { value } = e.target;
    setFloor(value);
  };

  // 화장실칸 생성
  const createGrid = (row, col) => {
    const data = [];
    for (let r = 1; r <= row; r++) {
      for (let c = 1; c <= col; c++) {
        data.push({ row: r, col: c, content: 0 });
      }
    }
    setStallData([...data]);
  };

  const handleStatusClick = (row, col) => {
    const updateData = stallData.map((data) => {
      if (data.row == row && data.col == col) {
        if ((data.content + 1) % 3 == 1) setStallCount(stallCount + 1);
        else if ((data.content + 1) % 3 == 2) setStallCount(stallCount - 1);
        return { ...data, content: (data.content + 1) % 3 };
      } else {
        return data;
      }
    });
    setStallData([...updateData]);
  };

  // 화장실 등록 (등록전에 list 목록 만들어줌.)
  const handleRestRoomSubmit = async () => {
    try {
      const registItems = document.querySelectorAll(".RegistItem");

      const list = Array.from(registItems).map((RegistItem) => {
        return {
          content: RegistItem.getAttribute("content"),
          row: RegistItem.getAttribute("row"),
          col: RegistItem.getAttribute("col"),
        };
      });

      const data = {
        buildingId: buildingId,
        floor: floor,
        gender: gender,
        height: gridSize.row,
        width: gridSize.col,
        list: list,
      };

      createRestroom(data);
      Swal.fire({
        icon: "success",
        title: "등록 완료!",
        text: `화장실 등록이 완료되었습니다.`,
        showCancelButton: false,
        confirmButtonText: "확인",
      });
      navigate(`/admin/toilet/${buildingId}`, { state: { floor: 1 } });
    } catch (e) {
      console.log(e);
    }
  };

  // 홈으로
  const handleHomeClick = () => {
    navigate(`/admin/1`);
  };

  return (
    <div className={styles.container}>
      <div className={styles.logoContainer}>
        <img
          className={styles.logo}
          src={process.env.PUBLIC_URL + `/assets/Logo.png`}
          onClick={handleHomeClick}
        />
        <div className={styles.text}>Admin Regist Page</div>
      </div>
      <div className={styles.registContainer}>
        <div className={styles.selectOption}>
          <div className={styles.sizeOption}>
            <input
              className={styles.sizeOptionInput}
              type="text"
              placeholder=" Row"
              name="row"
              value={gridSize.row}
              onChange={handleInputChange}
            ></input>
          </div>
          <div className={styles.sizeOption}>
            <input
              className={styles.sizeOptionInput}
              type="text"
              placeholder=" Col"
              name="col"
              value={gridSize.col}
              onChange={handleInputChange}
            ></input>
          </div>
          <div className={styles.sizeOption}>
            <select
              className={styles.sizeOptionSelect}
              onChange={handleGenderChange}
            >
              <option value={0}>남자</option>
              <option value={1}>여자</option>
            </select>
          </div>
          <div className={styles.sizeOption}>
            <select
              className={styles.sizeOptionSelect}
              onChange={handleFloorChange}
            >
              {numberOptions.map((number) => (
                <option key={number} value={number}>
                  {number}F
                </option>
              ))}
            </select>
          </div>
        </div>

        <div className={styles.registGrid}>
          {stallData.map((data, index) => (
            <RegistItem
              className="RegistItem"
              key={`${index}`}
              row={data.row}
              col={data.col}
              content={data.content}
              height={gridSize.row}
              width={gridSize.col}
              onClick={() => {
                handleStatusClick(data.row, data.col);
              }}
            />
          ))}
        </div>
        <div
          className={
            stallCount == 0 ? styles.submitBlock : styles.submitRestRoom
          }
          onClick={handleRestRoomSubmit}
        >
          등록하기
        </div>
      </div>
    </div>
  );
};

export default ToiletRegist;
