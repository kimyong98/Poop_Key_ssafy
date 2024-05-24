import styles from "./SignupPage.module.css";
import React, { useState } from "react";
import { emailCheck, passwordCheck } from "../../util/InputCheck";
import { useNavigate } from "react-router-dom";
import { createUser } from "../../util/API";

const SignupPage = () => {
  const [isEmailValid, setIsEmailValid] = useState(true);
  const [isPwValid, setIsPwValid] = useState(true);
  const [signupData, setSignupData] = useState({
    email: "",
    password: "",
  });
  const navigate = useNavigate();

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setSignupData({
      ...signupData,
      [name]: value,
    });

    if (name === "email") {
      if (emailCheck(value) || value === "") {
        setIsEmailValid(true);
        console.log("email 유효성 성공");
      } else {
        setIsEmailValid(false);
        console.log("email 유효성 실패");
      }
    }

    if (name === "password") {
      if (passwordCheck(value) || value === "") {
        setIsPwValid(true);
        console.log("pw 유효성 성공");
      } else {
        setIsPwValid(false);
        console.log("pw 유효성 실패");
      }
    }
  };

  const handleSignupSubmit = async (e) => {
    e.preventDefault();
    try {
      const response = await createUser(signupData);
      console.log(response.data);
      navigate("/");
    } catch (error) {
      console.log(error);
    }
  };

  return (
    <div className={styles.container}>
      <img
        className={styles.logo}
        src={process.env.PUBLIC_URL + `/assets/Logo.png`}
      />
      <form onSubmit={handleSignupSubmit}>
        <div>
          <input
            type="text"
            name="email"
            placeholder="Email"
            value={signupData.email}
            onChange={handleInputChange}
          />
        </div>
        <div className={styles.msg}>
          {!isEmailValid && `이메일 주소가 올바르지 않습니다.`}
        </div>

        <div>
          <input
            type="password"
            name="password"
            placeholder="Password"
            value={signupData.password}
            onChange={handleInputChange}
          />
        </div>
        <div className={styles.msg}>
          {!isPwValid &&
            `비밀번호는 영문,숫자,특수문자를 포함하여 8자 이상입니다.`}
        </div>

        <div className={styles.button}>
          <button type="submit">Sign Up</button>
        </div>
      </form>
    </div>
  );
};
export default SignupPage;
