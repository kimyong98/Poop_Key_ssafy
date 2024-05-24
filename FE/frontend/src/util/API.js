import axios from "axios";

// export const BASE_URL = "http://localhost:9999";

// export const Web_Socket_URL = "ws://localhost:9999/ws";

export const BASE_URL = "https://i10a104.p.ssafy.io";

export const Web_Socket_URL = "wss://i10a104.p.ssafy.io/ws";

//모든 건물 리스트
export const getAllBuilding = async () => {
    try {
      const response = await axios.get(`${BASE_URL}/api/user/building/selectall`);
      console.log(response.data);
      return response.data;
    } catch (error) {
      console.error(error);
    }
  };

// 건물 id를 넘겨줬을 때 전체 화장실 데이터를 준다.
export const getRestRoom = async (buildingId) => {
    try {
      const response = await axios.get(`${BASE_URL}/api/user/restroom/select`,{
        params: {
            buildingId: buildingId
        }
      });
      console.log(response);
      return response.data;
    } catch (error) {
      console.error("요청 실패", error);
    }
  };

// 화장실 id를 넘겨줬을 때 전체 화장실칸 데이터 받아오기
// [화장실칸1, 화장실칸2, 화장실칸3 ...]
export const getStall = async (restroomId) => {
    try {
      const response = await axios.get(`${BASE_URL}/api/api/tmptest1`,{
        params: {
            restroomId: restroomId
        }
      });
      console.log(response);
      return response.data;
    } catch (error) {
      console.error(error);
    }
  };

// buildingId를 넘겨주면 층별 화장실 혼잡도를 준다.
// 0: 원활 1: 혼잡 2: 포화
export const getCongestion = async (buildingId) => {
  try {
    //console.log(buildingId,"axios에 빌딩 id 잘 넘어오는중")
    const response = await axios.get(`${BASE_URL}/api/api/tmptest2`,{
      params: {
          buildingId: buildingId
      }
    });
    console.log(response);
    return response.data;
  } catch (error) {
    console.error(error);
  }
}

// 신고폼 제출 (stallId, content, seuserReportReasonlectㄴ)
export const createReportData = (data) => {
    axios.post(`${BASE_URL}/api/user/reports/regist`, data, {headers: { 'Authorization': `BEARER ${localStorage.getItem("UserToken")}`}})
    .then((res) => {
        console.log("신고 제출 성공")
    }).catch((e) => {
        console.log(e,"신고 제출 실패")
    })
}



// 회원가입
export const createUser = async (data) => {
  try {
    const response = await axios.post(`${BASE_URL}/register`,data);
    console.log(response);
  } catch (error) {
    console.log(error);
  }
}

// 로그인
export const userLogIn = async (data) => {
  try {
    //console.log(buildingId,"axios에 빌딩 id 잘 넘어오는중")
    const response = await axios.post(`${BASE_URL}/login`,data);
    console.log(response.data, "Token");
    return response.data;
  } catch (error) {
    console.log(error);
  }
}