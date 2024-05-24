import axios from "axios";

// export const BASE_URL = "http://localhost:9999";

export const BASE_URL = "https://i10a104.p.ssafy.io";

// 화장실 화장실 등록
export const createRestroom = (data) => {
    axios.post(`${BASE_URL}/api/master/restroom/insert`, data, {headers: { 'Authorization': `BEARER ${localStorage.getItem("UserToken")}`}})
    .then((res) => {
        console.log(res);
    }).catch((error) => {
        console.log(error);
    })
}

// 관리자 화장실칸 상태변경
export const updateStatus = (data) => {
    axios.put(`${BASE_URL}/api/master/stalls/update`, data)
    .then((res) => {
        console.log(res);
    }).catch((error) => {
        console.log(error);
    })
}

// 관리자 통계 조회 (건물명, 층수, 성별)
export const getStatistics = async (buildingName, floor, gender) => {
    try {
      const response = await axios.get(`${BASE_URL}/api/master/statistics`,{
        params: {
            buildingName: buildingName,
            floor: floor,
            gender: gender
        }
      });

      console.log(response.data);
      return response.data;
    } catch (error) {
      console.error("요청 실패", error);
    }
  }

  // 관리자 신고메일 리스트
export const getMailList = async (masterId) => {
    try {
      const response = await axios.get(`${BASE_URL}/api/master/reports/select`,{
        params: {
            masterId: masterId
        }
      });
      console.log(response.data, "신고 리스트");
      return response.data;
    } catch (error) {
      console.error("요청 실패", error);
    }
  }

  // 관리자 메일 읽음처리
  export const updateMailCheck = async (reportId) => {
    try{
      axios.put(`${BASE_URL}/api/master/reports/update`, null , {
          params: {
            reportId: reportId
          }
      })
    } catch(e){

    }
}

  // 관리자 신고메일 디테일 (화장실칸 위치를 특정)
  export const getMailLocation = async (stallId) => {
    try {
      const response = await axios.get(`${BASE_URL}/api/api/tmptest4`,{
        params: {
          stallId: stallId
        }
      });
      console.log(response);
      return response.data;
    } catch (error) {
      console.error(error);
    }
  }