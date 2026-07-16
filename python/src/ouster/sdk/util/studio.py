import json
import requests
import secrets
import base64
import hashlib
import webbrowser
import socketserver
import configparser
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from requests.adapters import HTTPAdapter
from urllib.parse import parse_qs, urlparse, urlencode


def save_tokens(keycloak_url, realm_name, client_id, base_url, token):
    home_directory = Path.home()
    config_directory = home_directory / ".config" / "ouster"
    config_directory.mkdir(parents=True, exist_ok=True)
    config_file_path = config_directory / "application_default_credentials.json"

    token_str = json.dumps(token)

    config = configparser.ConfigParser()
    config["Keycloak"] = {
        "KeycloakUrl": keycloak_url,
        "RealmName": realm_name,
        "ClientId": client_id,
        "BaseUrl": base_url,
        "Token": token_str,  # Store the JSON string representation of the token
    }

    with open(config_file_path, "w") as configfile:
        config.write(configfile)


def delete_tokens_file():
    home_directory = Path.home()
    config_file_path = (
        home_directory / ".config" / "ouster" / "application_default_credentials.json"
    )

    if config_file_path.exists():
        config_file_path.unlink()


def load_tokens():
    home_directory = Path.home()
    config_directory = home_directory / ".config" / "ouster"
    config_file_path = config_directory / "application_default_credentials.json"

    config = configparser.ConfigParser()
    config.read(config_file_path)

    if "Keycloak" in config:
        keycloak_url = config["Keycloak"].get("KeycloakUrl")
        realm_name = config["Keycloak"].get("RealmName")
        client_id = config["Keycloak"].get("ClientId")
        base_url = config["Keycloak"].get("BaseUrl")
        token_str = config["Keycloak"].get("Token")
        token = json.loads(token_str)
        return keycloak_url, realm_name, client_id, base_url, token
    else:
        return None


class KeycloakPKCETokenProviderV1:
    def code_verifier(self):
        random = secrets.token_bytes(64)
        code_verifier = base64.b64encode(random, b"-_").decode().replace("=", "")
        return code_verifier

    def _code_challenge(self, code_verifier):
        m = hashlib.sha256()
        m.update(code_verifier.encode())
        d = m.digest()
        code_challenge = base64.b64encode(d, b"-_").decode().replace("=", "")
        return code_challenge

    def __init__(self, client_id, base_url, keycloak_url, realm_name, organization_id):
        self.client_id = client_id
        self._current_token = None
        self.token = None
        self.realm_name = realm_name
        self.auth_url = None
        self.base_url = base_url
        self.keycloak_url = keycloak_url
        self.code_verifier = self.code_verifier()
        self.code_challenge = self._code_challenge(self.code_verifier)
        self.organization_id = organization_id

        self.redirect_uri = "http://localhost:5050/callback"

    def current(self):
        result = load_tokens()
        tokens = None
        if result:
            _, _, _, _, tokens = result
        if tokens is not None and "access_token" in tokens:
            self.token = tokens["access_token"]
            return tokens["access_token"]
        return None

    def __next__(self):
        return None

    def auth(self):
        """Call to refresh the tokens and initiate the authorization process."""
        self.token = self.get_token()
        save_tokens(
            keycloak_url=self.keycloak_url,
            realm_name=self.realm_name,
            client_id=self.client_id,
            base_url=self.base_url,
            token=self.token,
        )
        return self.token

    def make_handler(self, outer_self):
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                query = parse_qs(urlparse(self.path).query)
                code = query.get("code", [None])[0]
                if code:
                    outer_self._current_token = outer_self.exchange_code_for_token(code)
                self.send_response(200)
                self.end_headers()
                if "access_token" in outer_self._current_token:
                    self.wfile.write(
                        b"You were authenticated and the authentication was recorded in Data app"
                        b" client. Please return to the console."
                    )
                else:
                    self.wfile.write(b"An error occurred while retrieving the token")

        return Handler

    def get_token(self):
        self.init_tokens()

        # first try and refresh the token
        try:
            result = load_tokens()
            tokens = None
            if result:
                _, _, _, _, tokens = result
            if tokens is not None and "refresh_token" in tokens:
                self._current_token = self.refresh_tokens()
                if self._current_token is not None:
                    return self._current_token
        except requests.HTTPError:
            pass

        auth_url = self.get_authorization_url()
        print(f"Opening the following URL to authorize the application: {auth_url}")
        webbrowser.open(auth_url)
        # Start a simple HTTP server to handle the callback
        with socketserver.TCPServer(("", 5050), self.make_handler(self)) as httpd:
            httpd.handle_request()
        return self._current_token

    def init_tokens(self, client=None):
        """Initialize the token provider."""
        auth_url = self.keycloak_url + "realms/" + self.realm_name + "/protocol/openid-connect/auth"

        params = {
            "response_type": "code",
            "scope": "openid email profile",
            "client_id": self.client_id,
            "code_challenge_method": "S256",
            "code_challenge": self.code_challenge,
            "redirect_uri": self.redirect_uri,
        }

        self.auth_url = f"{auth_url}?{urlencode(params)}"

    def exchange_code_for_token(self, code):
        token_url = self.keycloak_url + "realms/" + self.realm_name + "/protocol/openid-connect/token"
        token_params = {
            "client_id": self.client_id,
            "grant_type": "authorization_code",
            "code": code,
            "redirect_uri": self.redirect_uri,
            "code_verifier": self.code_verifier,
        }
        token_response = requests.post(token_url, data=token_params)
        token = token_response.json()

        return token

    def refresh_tokens(self):
        result = load_tokens()
        tokens = None
        if result:
            _, _, _, _, tokens = result

        if tokens is None or "refresh_token" not in tokens:
            return None

        refresh_token = tokens["refresh_token"]
        payload = {
            "grant_type": "refresh_token",
            "client_id": self.client_id,
            "refresh_token": refresh_token
        }

        refresh_url = self.keycloak_url + "realms/" + self.realm_name + "/protocol/openid-connect/token"
        response = requests.post(refresh_url, data=payload)
        response.raise_for_status()

        token_data = response.json()
        return token_data

    def get_authorization_url(self):
        return self.auth_url


class RetryableException(Exception):
    """Exception used for driving retries in the requests. Internal use."""
    pass


class UnAuthorizedException(Exception):
    """Exception used when user's Access token is invalid. Internal use."""
    pass


class TokenExpiredException(Exception):
    pass


class BearerTokenRedirectAdapter(HTTPAdapter):
    """Custom adapter to ensure the bearer token is passed along in redirects."""

    def __init__(self, token_provider, *args, **kwargs):
        self.token_provider = token_provider
        super().__init__(*args, **kwargs)

    def send(self, request, **kwargs):
        # Attach the Authorization header before sending the request
        token = self.token_provider.current()
        if token:
            request.headers["Authorization"] = f"Bearer {token}"

        response = super().send(request, **kwargs)

        # Handle redirection and re-apply the token if necessary
        if response.is_redirect:
            request.headers["Authorization"] = f"Bearer {self.token_provider.current()}"

        return response


def get_common_headers(token_provider):
    if token_provider and token_provider.current() is not None:
        return {"Authorization": f"Bearer {token_provider.current()}"}
    else:
        return {}


def check_and_raise(response, token_provider):
    if response.status_code == 401:
        token = next(token_provider)
        if token is None:
            raise UnAuthorizedException()

        raise RetryableException()

    if response.status_code in [429, 503]:
        # If we think something is wrong, we say we retry
        raise RetryableException()
    return response


def get_json(response):
    """Helper for getting json out of response. If the response is not json or the
    json is invalid it returns None

    Args:
        response (Response): Response from the API

    Returns:
        dict: json response
    """
    if response.status_code == 204:
        return None

    try:
        return response.json()
    except json.decoder.JSONDecodeError:
        return None


TIMEOUT = 15


def get(url, *, params=None, token_provider):
    session = get_session_with_redirect(token_provider)

    def call():
        response = session.get(
            url,
            params=params,
            timeout=TIMEOUT,
            headers=get_common_headers(token_provider),
        )
        result = None
        try:
            result = check_and_raise(response, token_provider)
            return result
        except UnAuthorizedException:
            return response

    return call()


def post(url, *, data, token_provider):
    session = get_session_with_redirect(token_provider)

    def call():
        response = session.post(
            url, json=data, timeout=TIMEOUT, headers=get_common_headers(token_provider)
        )
        result = check_and_raise(response, token_provider)
        return result

    return call()


# Function to set up session with redirection and retry logic
def get_session_with_redirect(token_provider):
    session = requests.Session()
    adapter = BearerTokenRedirectAdapter(token_provider)
    # Mount the adapter to the session
    session.mount("http://", adapter)
    session.mount("https://", adapter)

    return session


class DataAppFile:
    def __init__(self, id, recording_url, client, output_type, type):
        self.id = id
        self.recording_url = recording_url
        self.client = client
        self.output_type = output_type
        self.type = type

    @property
    def url(self):
        return self.recording_url + f"/output/files/{self.id}"

    def get_signed_url(self):
        url = self.url + "/signed_url"
        data = self.client.post(url, data={})
        return data.json()["url"]


class DataAppClient:
    def __init__(self, org_id):
        self.base_url = "https://data.ouster.dev/"
        self.organization_id = org_id
        self.api_url = self.base_url + f"api/v1/organizations/{self.organization_id}/"

        self.token_provider = KeycloakPKCETokenProviderV1(
                client_id="pkce-client",
                realm_name="Ouster",
                keycloak_url="https://id.ouster.com/",
                base_url=self.base_url,
                organization_id=self.organization_id)

    def get(self, url, *, params=None):
        res = get(url, params=params, token_provider=self.token_provider)
        if res.status_code == 401:
            raise UnAuthorizedException()
        if res.status_code == 404:
            raise RuntimeError("URL not found")
        return res

    def post(self, url, *, data=None):
        res = post(url, data=data, token_provider=self.token_provider)
        if res.status_code == 401:
            raise UnAuthorizedException()
        if res.status_code == 404:
            raise RuntimeError("URL not found")
        return res

    def get_url_from_id(self, id):
        return self.api_url + f"recordings/{id}"

    def auth(self):
        """Authenticates"""
        self.token_provider.auth()

    def get_output_files(self, id):
        """Get output file information about a specific drive."""
        url = self.get_url_from_id(id)
        files_output_url = url + "/output/files/"
        out = self.get(files_output_url)
        files = out.json()
        l = []
        for file in files:
            l.append(DataAppFile(file["id"], url, self, file["output_type"], file["type"]))

        return l

    def get_drive(self, id):
        """Gets information about a specific drive."""
        url = self.get_url_from_id(id)
        # If a drive was specified, we should have an id by now
        # and we get back just one drive
        return self.get(url).json()


def get_url_from_share_link(link: str):
    recording_id = link.split("?")[0].split("/")[-1]
    recording_url = "https://studio.ouster.com/api/v1/published_recordings/" + recording_id
    objects = requests.get(recording_url + "/output/files/", timeout=TIMEOUT).json()
    for obj in objects:
        if obj["output_type"] == "osf" and obj["type"] == "osf":
            data = {}
            data["object_id"] = obj["path"]
            result = requests.post(recording_url + "/signings/", json=data, timeout=TIMEOUT).json()
            return result["url"]
    return None


def get_osf_from_data_app_link(link: str):
    """Get OSF download link for a data-app drive link"""
    if "/share/" in link:
        # support for share links with no org id
        return get_url_from_share_link(link)

    split_path = link.split("?")
    path = split_path[0]
    drive_id = path.split("/")[-1]
    org_id = ""
    if "/organization/" in link:
        # support for links with org id in the path
        org_id = path.split("/")[-3]
    elif len(split_path) > 1:
        # support for links with org id as a parameter
        params = split_path[1]
        for arg in params.split("&"):
            if "=" not in arg:
                continue
            key = arg.split("=")[0].lower()
            value = arg.split("=")[1]
            if key == "orgid":
                org_id = value
                break
    if org_id == "":
        raise RuntimeError("No organization id found in URL")
    client = DataAppClient(org_id = org_id)
    try:
        files = client.get_output_files(drive_id)
    except UnAuthorizedException:
        client.auth()
        files = client.get_output_files(drive_id)
    for file in files:
        if file.output_type != "osf" or file.type != "osf":
            continue
        return file.get_signed_url()
    return None
