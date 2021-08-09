
 /**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2016 Thomas Perl <m@thp.io>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/


#include "psmoveapi.h"

#include "psmove_port.h"
#include "psmove_private.h"
#include "daemon/moved_monitor.h"

#include <vector>
#include <map>
#include <string>

#include <stdlib.h>
#include <string.h>

namespace {

struct ControllerGlue {
    ControllerGlue(int index, const std::string &serial);
    ~ControllerGlue();

    void add_handle(PSMove *handle);
    void update_connection_flags();

    ControllerGlue(const ControllerGlue &other) = delete;
    Controller &operator=(const ControllerGlue &other) = delete;

    PSMove *read_move() { return move_bluetooth; }
    PSMove *write_move() { return move_usb ? move_usb : move_bluetooth; }

    PSMove *move_bluetooth;
    PSMove *move_usb;
    std::string serial;
    struct Controller controller;
    bool connected;
    bool api_connected;
};

#ifdef _WIN32
class DeviceNotificationWindow;
#endif

struct PSMoveAPI {
    PSMoveAPI(EventReceiver *receiver, void *user_data);
    ~PSMoveAPI();

    void update();

    static void on_monitor_event(enum MonitorEvent event, enum MonitorEventDeviceType device_type, const char *path, const wchar_t *serial, void *user_data);

    EventReceiver *receiver;
    void *user_data;
    std::vector<ControllerGlue *> controllers;
#ifdef _WIN32
	DeviceNotificationWindow *monitor;
#else
    moved_monitor *monitor;
#endif
};

PSMoveAPI *
g_psmove_api = nullptr;

#ifdef _WIN32
# include <windows.h>
# include <wrl/client.h>
# include <dbt.h>
# include <bluetoothapis.h>
# include <hidapi.h>

# ifdef _MSC_VER
// Disable 64-bit portability warnings
#  pragma warning(disable: 4312 )
// Disable 'deprecated function' warnings
#  pragma warning(disable: 4996)
# endif // _MSC_VER

#define NUM_PSMOVE_PIDS \
    ((sizeof(PSMOVE_PIDS) / sizeof(PSMOVE_PIDS[0])) - 1)

static int
PSMOVE_PIDS[] = {
	PSMOVE_PID,
	PSMOVE_PS4_PID,
	0,
};

class DeviceNotificationWindow
{
public:
	DeviceNotificationWindow(void *context);
	~DeviceNotificationWindow();

protected:
	/** Open/close window **/
	bool open();
	void close();

	/** Wait until window is closed **/
	void waitForClose();

private:
	/** Notification window message handler **/
	static LRESULT CALLBACK wndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	/** Device notification handler **/
	void handleBluetoothConnect(BTH_ADDR bthAddress, bool connect);

protected:
	/** Thread entry point **/
	static DWORD WINAPI wndThreadProc(void *arg);

	/** Start thread **/
	bool startThread();

	/** Stop thread **/
	bool stopThread();

	/** Create/destroy window **/
	bool createWindow();
	void destroyWindow();

	/** Post message to window **/
	void postMessage(UINT uMsg, WPARAM wParam = 0, LPARAM lParam = 0)
	{
		if (window != nullptr)
		{
			PostMessage(window, uMsg, wParam, lParam);
		}
	}

private:
	/* User data */
	void *user_data;

	/* Window and windows message thread handles */
	HWND window;
	HANDLE thread;
	HANDLE semaphore;

	/* Device notification handle */
	HANDLE radio;
	HDEVNOTIFY notification;
};

/** Constructor/destructor **/
DeviceNotificationWindow::DeviceNotificationWindow(void *context)
	: user_data(context)
	, window(NULL)
	, thread(INVALID_HANDLE_VALUE)
	, semaphore(INVALID_HANDLE_VALUE)
	, radio(INVALID_HANDLE_VALUE)
	, notification(NULL)
{
	open();
}

DeviceNotificationWindow::~DeviceNotificationWindow()
{
	close();
}

bool DeviceNotificationWindow::open()
{
	// Start window message thread
	return startThread();
}

void DeviceNotificationWindow::close()
{
	stopThread();
	destroyWindow();
}

/** Wait until window is closed **/
void DeviceNotificationWindow::waitForClose()
{
	// Wait for thread to quit
	WaitForSingleObject(thread, INFINITE);
}

/** Start thread **/
bool DeviceNotificationWindow::startThread()
{
	// Not already started?
	if (thread == INVALID_HANDLE_VALUE)
	{
		// Create window initialisation semaphore
		semaphore = CreateSemaphore(NULL, 0, 1, NULL);

		// Start thread
		thread = CreateThread(NULL, 0, wndThreadProc, (void*)this, 0, NULL);
		if (thread != INVALID_HANDLE_VALUE)
		{
			// Wait for initialisation to complete
			WaitForSingleObject(semaphore, 3000);
			CloseHandle(semaphore);
			semaphore = INVALID_HANDLE_VALUE;
		}
	}

	return true;
}

/** Stop thread **/
bool DeviceNotificationWindow::stopThread()
{
	// Thread running?
	if (thread != INVALID_HANDLE_VALUE)
	{
		// Send quit message to thread
		postMessage(WM_QUIT);
		waitForClose();
	}

	return true;
}

/** Thread entry point **/
DWORD WINAPI DeviceNotificationWindow::wndThreadProc(void *arg)
{
	DeviceNotificationWindow *thiz = reinterpret_cast<DeviceNotificationWindow*>(arg);
	if (thiz != nullptr)
	{
		// Create window
		if (thiz->createWindow())
		{
			// Get handle to first bluetooth radio 
			BLUETOOTH_FIND_RADIO_PARAMS radio_params;
			radio_params.dwSize = sizeof(BLUETOOTH_FIND_RADIO_PARAMS);
			HBLUETOOTH_RADIO_FIND hFind = BluetoothFindFirstRadio(&radio_params, &thiz->radio);
			if (hFind != NULL)
			{
				// Close find
				BluetoothFindRadioClose(hFind);

				// Register for device notifications
				DEV_BROADCAST_HANDLE NotificationFilter;
				ZeroMemory(&NotificationFilter, sizeof(NotificationFilter));
				NotificationFilter.dbch_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
				NotificationFilter.dbch_devicetype = DBT_DEVTYP_HANDLE;
				NotificationFilter.dbch_handle = thiz->radio;

				thiz->notification = RegisterDeviceNotification(
					thiz->window,     // events recipient
					&NotificationFilter,        // type of device
					DEVICE_NOTIFY_WINDOW_HANDLE // type of recipient handle
				);

				if (thiz->notification != NULL)
				{
					// Signal that thread has initialised
					if (thiz->semaphore != INVALID_HANDLE_VALUE)
					{
						ReleaseSemaphore(thiz->semaphore, 1, NULL);
					}

					// Pump messages
					MSG message;
					while (GetMessage(&message, NULL, 0, 0))
					{
						// Process message
						TranslateMessage(&message);
						DispatchMessage(&message);
					}

					// Unregister for device notifications
					UnregisterDeviceNotification(thiz->notification);
					thiz->notification = NULL;
				}

				// Close bluetooth radio handle
				CloseHandle(thiz->radio);
				thiz->radio = INVALID_HANDLE_VALUE;
			}

			// Clear class pointer in window data
			SetWindowLongPtr(thiz->window, GWLP_USERDATA, (LONG_PTR)0);

			// Destroy window
			thiz->destroyWindow();
		}
	}
	else
	{
		// Signal that thread has failed to initialise
		if (thiz->semaphore != INVALID_HANDLE_VALUE)
		{
			ReleaseSemaphore(thiz->semaphore, 1, NULL);
		}
	}

	thiz->thread = INVALID_HANDLE_VALUE;

	return 0;
}

/** Create output window **/
bool DeviceNotificationWindow::createWindow()
{
	if (window == nullptr)
	{
		WNDCLASS windowClass;

		// Set up window class structure
		memset(&windowClass, 0, sizeof(WNDCLASS));
		windowClass.style = CS_HREDRAW | CS_VREDRAW;
		windowClass.lpfnWndProc = wndProc;
		windowClass.hInstance = GetModuleHandle(NULL);
		windowClass.hbrBackground = (HBRUSH)(COLOR_WINDOW + 2);
		windowClass.lpszClassName = TEXT("PsMoveDevNotifyWnd");

		// Register window class
		if (!RegisterClass(&windowClass))
		{
			// Not already registered?
			if (GetLastError() != ERROR_CLASS_ALREADY_EXISTS)
			{
				return false;
			}
		}

		// Create window
		if ((window = CreateWindowEx(0,
			windowClass.lpszClassName, TEXT(""), 0,
			0, 0, 0, 0, NULL, NULL, windowClass.hInstance, NULL)) == nullptr)
		{
			return false;
		}

		// Save class pointer in window data
		SetWindowLongPtr(window, GWLP_USERDATA, (LONG_PTR)this);

		// Show window
		ShowWindow(window, SW_HIDE);
		UpdateWindow(window);

		// Create message queue
		MSG message;
		PeekMessage(&message, NULL, 0, 0, 0);
	}

	return true;
}

/** Destroy window **/
void DeviceNotificationWindow::destroyWindow()
{
	if (window != NULL)
	{
		// Destroy window
		DestroyWindow(window);
		window = NULL;
	}
}

static int
compare_hid_device_info_ptr(const void *a, const void *b)
{
	struct hid_device_info *dev_a = *(struct hid_device_info **)a;
	struct hid_device_info *dev_b = *(struct hid_device_info **)b;

	if ((dev_a->serial_number != NULL) && (wcslen(dev_a->serial_number) != 0) &&
		(dev_b->serial_number != NULL) && (wcslen(dev_b->serial_number) != 0)) {
		return wcscmp(dev_a->serial_number, dev_b->serial_number);
	}

	if (dev_a->path != NULL && dev_b->path != NULL) {
		return strcmp(dev_a->path, dev_b->path);
	}
	return 0;
}

/** Device notification handler **/
void DeviceNotificationWindow::handleBluetoothConnect(BTH_ADDR bthAddress, bool connect)
{
	wchar_t serial_number[13];
	unsigned char *address = (unsigned char*)&bthAddress;
	_swprintf(serial_number, L"%02x%02x%02x%02x%02x%02x", address[5], address[4], address[3], address[2], address[1], address[0]);

	if (connect) {
		// Give HID device time to register
		Sleep(500);
	}

	/* From psmove.c */
	struct hid_device_info *move_hid_devices[NUM_PSMOVE_PIDS], *devs, *cur_dev;

	// enumerate matching HID devices
	for (unsigned int i = 0; i < NUM_PSMOVE_PIDS; i++) {
		// NOTE: hidapi returns NULL for PIDs that were not found
		move_hid_devices[i] = hid_enumerate(PSMOVE_VID, PSMOVE_PIDS[i]);
	}

	// Count available devices
	int available = 0;
	int i;
	for (i = 0; i < NUM_PSMOVE_PIDS; i++) {
		for (cur_dev = move_hid_devices[i]; cur_dev != NULL; cur_dev = cur_dev->next, available++);
	}

	// Sort list of devices to have stable ordering of devices
	int n = 0;
	struct hid_device_info **devs_sorted = (struct hid_device_info **) calloc(available, sizeof(struct hid_device_info *));
	for (i = 0; i < NUM_PSMOVE_PIDS; i++) {
		cur_dev = move_hid_devices[i];
		while (cur_dev && (n < available)) {
			devs_sorted[n] = cur_dev;
			cur_dev = cur_dev->next;
			n++;
		}
	}
	qsort((void *)devs_sorted, available, sizeof(struct hid_device_info *), compare_hid_device_info_ptr);

	int count = 0;
	for (i = 0; i < available; i++) {
		cur_dev = devs_sorted[i];

		if (strstr(cur_dev->path, "&col01#") != NULL) {
			// Found matching serial number?
			if (wcsicmp(cur_dev->serial_number, serial_number) == 0) {
				// Call monitor event handler
				PSMoveAPI::on_monitor_event(connect ? EVENT_DEVICE_ADDED : EVENT_DEVICE_REMOVED, EVENT_DEVICE_TYPE_BLUETOOTH, cur_dev->path, cur_dev->serial_number, user_data);
			}
		}
		else {
			count--;
		}

		count++;
	}

	free(devs_sorted);

	// free HID device enumerations
	for (i = 0; i < NUM_PSMOVE_PIDS; i++) {
		devs = move_hid_devices[i];
		if (devs) {
			hid_free_enumeration(devs);
		}
	}
}

/** Notification window message handler **/
LRESULT CALLBACK DeviceNotificationWindow::wndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg)
	{
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;

		// Device change notification?
	case WM_DEVICECHANGE:
		// Custom event?
		if (wParam == DBT_CUSTOMEVENT)
		{
			DEV_BROADCAST_HDR *data = (DEV_BROADCAST_HDR*)lParam;
			if (data != NULL && data->dbch_devicetype == DBT_DEVTYP_HANDLE)
			{
				// Bluetooth HCI event?
				DEV_BROADCAST_HANDLE *handleData = (DEV_BROADCAST_HANDLE*)lParam;
				if (IsEqualGUID(handleData->dbch_eventguid, GUID_BLUETOOTH_HCI_EVENT))
				{
					BTH_HCI_EVENT_INFO *hciData = (BTH_HCI_EVENT_INFO*)&handleData->dbch_data;
					DeviceNotificationWindow *thiz = (DeviceNotificationWindow*)GetWindowLongPtr(hWnd, GWLP_USERDATA);
					if (thiz != NULL)
					{
						// Handle event
						thiz->handleBluetoothConnect(hciData->bthAddress, hciData->connected != 0);
					}
				}
			}
			return 0;
		}
		// Fall through...

	default:
		// Call default window message handler
		return DefWindowProc(hWnd, uMsg, wParam, lParam);
	}
}
#endif
}; // end anonymous namespace

ControllerGlue::ControllerGlue(int index, const std::string &serial)
    : move_bluetooth(nullptr)
    , move_usb(nullptr)
    , serial(serial)
    , controller()
    , connected(false)
    , api_connected(false)
{
    memset(&controller, 0, sizeof(controller));
    controller.index = index;
    controller.serial = this->serial.c_str();
}

void
ControllerGlue::add_handle(PSMove *handle)
{
    if (psmove_connection_type(handle) == Conn_USB) {
        if (move_usb != nullptr) {
            psmove_WARNING("USB handle already exists for this controller");
            psmove_disconnect(move_usb);
        }

        move_usb = handle;
    } else {
        if (move_bluetooth != nullptr) {
            psmove_WARNING("Bluetooth handle already exists for this controller -- leaking");
            psmove_disconnect(move_bluetooth);
        }

        move_bluetooth = handle;
    }

	controller.move = read_move();
}

void
ControllerGlue::update_connection_flags()
{
    controller.usb = (move_usb != nullptr);
    controller.bluetooth = (move_bluetooth != nullptr);

    if (controller.usb && !controller.bluetooth) {
        controller.battery = Batt_CHARGING;
    }

    connected = (controller.usb || controller.bluetooth);
}

ControllerGlue::~ControllerGlue()
{
    if (move_bluetooth != nullptr) {
        psmove_disconnect(move_bluetooth);
    }
    if (move_usb != nullptr) {
        psmove_disconnect(move_usb);
    }
}

PSMoveAPI::PSMoveAPI(EventReceiver *receiver, void *user_data)
    : receiver(receiver)
    , user_data(user_data)
    , controllers()
    , monitor(nullptr)
{
    std::map<std::string, std::vector<PSMove *>> moves;

    int n = psmove_count_connected();
    for (int i=0; i<n; i++) {
        PSMove *move = psmove_connect_by_id(i);
        if (!move) {
            psmove_WARNING("Failed to connect to controller #%d", i);
            continue;
        }

        char *tmp = psmove_get_serial(move);
        if (!tmp) {
            psmove_WARNING("Failed to get serial for controller #%d", i);
            continue;
        }

        std::string serial(tmp);
        psmove_free_mem(tmp);

        moves[serial].emplace_back(move);
    }

    int i = 0;
    for (auto &kv: moves) {
        auto c = new ControllerGlue(i++, kv.first);
        for (auto &handle: kv.second) {
            c->add_handle(handle);
        }
        controllers.emplace_back(c);
    }

#ifdef _WIN32
	monitor = new DeviceNotificationWindow(this);
#else
    monitor = moved_monitor_new(PSMoveAPI::on_monitor_event, this);
#endif
}

PSMoveAPI::~PSMoveAPI()
{
#ifdef _WIN32
	delete monitor;
#else
    moved_monitor_free(monitor);
#endif

    for (auto &c: controllers) {
        if (c->api_connected) {
            if (receiver->disconnect != nullptr) {
                // Send disconnect event
                receiver->disconnect(&c->controller, user_data);
            }

            c->api_connected = false;
        }

        delete c;
    }
}

void
PSMoveAPI::update()
{
#ifndef _WIN32
    if (moved_monitor_get_fd(monitor) == -1) {
        moved_monitor_poll(monitor);
    } else {
        struct pollfd pfd;
        pfd.fd = moved_monitor_get_fd(monitor);
        pfd.events = POLLIN;
        while (poll(&pfd, 1, 0) > 0) {
            moved_monitor_poll(monitor);
        }
    }
#endif

    for (auto &c: controllers) {
        c->update_connection_flags();

        if (c->connected && !c->api_connected) {
            if (receiver->connect != nullptr) {
                // Send initial connect event
                receiver->connect(&c->controller, user_data);
            }

            c->api_connected = c->connected;
        }

        if (!c->connected && c->api_connected) {
            if (receiver->disconnect != nullptr) {
                // Send disconnect event
                receiver->disconnect(&c->controller, user_data);
            }
            c->api_connected = c->connected;
        }

        if (!c->connected) {
            // Done with this controller (TODO: Can we check reconnect?)
            continue;
        }

        auto read_move = c->read_move();
        if (read_move == nullptr) {
            // We don't have a handle that supports reading (USB-only connection);
            // we call update exactly once (no new data will be reported), so that
            // the update method can change the LED and rumble values
            if (receiver->update != nullptr) {
                receiver->update(&c->controller, user_data);
            }
        } else {
            while (psmove_poll(read_move)) {
                if (receiver->update != nullptr) {
                    int previous = c->controller.buttons;
                    c->controller.buttons = psmove_get_buttons(read_move);
                    c->controller.pressed = c->controller.buttons & ~previous;
                    c->controller.released = previous & ~c->controller.buttons;
                    c->controller.trigger = float(psmove_get_trigger(read_move)) / 255.f;

                    psmove_get_accelerometer_frame(read_move, Frame_SecondHalf,
                            &c->controller.accelerometer.x,
                            &c->controller.accelerometer.y,
                            &c->controller.accelerometer.z);
                    psmove_get_gyroscope_frame(read_move, Frame_SecondHalf,
                            &c->controller.gyroscope.x,
                            &c->controller.gyroscope.y,
                            &c->controller.gyroscope.z);
                    psmove_get_magnetometer_vector(read_move,
                            &c->controller.magnetometer.x,
                            &c->controller.magnetometer.y,
                            &c->controller.magnetometer.z);
                    c->controller.battery = psmove_get_battery(read_move);

                    receiver->update(&c->controller, user_data);
                }
            }
        }

        auto write_move = c->write_move();
        if (write_move != nullptr) {
            psmove_set_leds(write_move,
                    uint32_t(255 * c->controller.color.r),
                    uint32_t(255 * c->controller.color.g),
                    uint32_t(255 * c->controller.color.b));
            psmove_set_rumble(write_move, uint32_t(255 * c->controller.rumble));
            psmove_update_leds(write_move);
        }
    }
}

void
PSMoveAPI::on_monitor_event(enum MonitorEvent event, enum MonitorEventDeviceType device_type, const char *path, const wchar_t *serial, void *user_data)
{
    auto self = static_cast<PSMoveAPI *>(user_data);

    switch (event) {
        case EVENT_DEVICE_ADDED:
            {
                psmove_DEBUG("on_monitor_event(event=EVENT_DEVICE_ADDED, device_type=0x%08x, path=\"%s\", serial=%p)",
                       device_type, path, serial);

                for (auto &c: self->controllers) {
                    if ((c->move_bluetooth != nullptr && strcmp(_psmove_get_device_path(c->move_bluetooth), path) == 0) ||
                            (c->move_usb != nullptr && strcmp(_psmove_get_device_path(c->move_usb), path) == 0)) {
                        psmove_WARNING("This controller is already active!");
                        return;
                    }
                }

                // TODO: FIXME: This should use the device's actual USB product ID.
                // HACK: We rely on this invalid PID being translated to a
                //       valid controller model (the old ZCM1, by default).
                unsigned short pid = 0;
                PSMove *move = psmove_connect_internal(serial, path, -1, pid);
                if (move == nullptr) {
                    psmove_CRITICAL("Cannot open move for retrieving serial!");
                    return;
                }

                char *serial_number = psmove_get_serial(move);

                bool found = false;
                for (auto &c: self->controllers) {
                    if (strcmp(c->serial.c_str(), serial_number) == 0) {
                        c->add_handle(move);
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    auto c = new ControllerGlue(self->controllers.size(), std::string(serial_number));
                    c->add_handle(move);
                    self->controllers.emplace_back(c);
                }

                psmove_free_mem(serial_number);
            }
            break;
        case EVENT_DEVICE_REMOVED:
            {
                psmove_DEBUG("on_monitor_event(event=EVENT_DEVICE_REMOVED, device_type=0x%08x, path=\"%s\", serial=%p)",
                       device_type, path, serial);

                bool found = false;
                for (auto &c: self->controllers) {
                    if (c->move_bluetooth != nullptr) {
                        const char *devpath = _psmove_get_device_path(c->move_bluetooth);
                        if (devpath != nullptr && strcmp(devpath, path) == 0) {
                            psmove_disconnect(c->move_bluetooth), c->move_bluetooth = nullptr;
                            found = true;
                            break;
                        }
                    }

                    if (c->move_usb != nullptr) {
                        const char *devpath = _psmove_get_device_path(c->move_usb);
                        if (devpath != nullptr && strcmp(devpath, path) == 0) {
                            psmove_disconnect(c->move_usb), c->move_usb = nullptr;
                            found = true;
                            break;
                        }
                    }
                }

                if (!found) {
                    psmove_CRITICAL("Did not find device for removal\n");
                }
            }
            break;
        default:
            psmove_CRITICAL("Invalid event");
            break;
    }

}

void
psmoveapi_init(EventReceiver *receiver, void *user_data)
{
    if (g_psmove_api == nullptr) {
        g_psmove_api = new PSMoveAPI(receiver, user_data);
    }
}

void
psmoveapi_update()
{
    if (g_psmove_api != nullptr) {
        g_psmove_api->update();
    }
}

void
psmoveapi_quit()
{
    delete g_psmove_api;
    g_psmove_api = nullptr;
}
