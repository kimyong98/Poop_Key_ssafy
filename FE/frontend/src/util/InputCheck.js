export function emailCheck(email) {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
}

export function nameCheck(name) {
    const nameRegex = /^[^\s]{2,}$/;
    return nameRegex.test(name);
}

export function telCheck(tel) {
    const telRegex = /^(01[016789]{1})-?[0-9]{3,4}-?[0-9]{4}$/;
    return telRegex.test(tel);
}

export function passwordCheck(password) {
    const passwordRegex = /^(?=.*[a-z])(?=.*[0-9])(?=.*[!@#$%^&*()_+{}\[\]:;<>,.?~\\/-]).{8,}$/;
    return passwordRegex.test(password);
}

export function passwordReCheck(password, passwordRe) {
    return password === passwordRe;
}