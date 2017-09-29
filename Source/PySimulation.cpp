#include "PyComponent.h"
#include "PyInterface.h"
#include "PySimulation.h"

#include <cfloat>
#include <iostream>

#ifdef __linux__
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>
#endif

using namespace DPsim;

void PySimulation::simThreadFunction(PySimulation* pySim) {
	bool notDone = true;

#ifdef __linux__
	if (pySim->rt) {
		simThreadFunctionRT(pySim);
		return;
	}
#endif
	std::unique_lock<std::mutex> lk(*pySim->mut, std::defer_lock);
	pySim->numStep = 0;
	while (pySim->running && notDone) {
		notDone = pySim->sim->step(*pySim->log, *pySim->llog, *pySim->rlog);
		pySim->numStep++;
		pySim->sim->increaseByTimeStep();
		if (pySim->sigPause) {
			lk.lock();
			pySim->state = StatePaused;
			pySim->cond->notify_one();
			pySim->cond->wait(lk);
			pySim->state = StateRunning;
			lk.unlock();
		}
	}
	lk.lock();
	pySim->state = StateDone;
	pySim->cond->notify_one();
}

#ifdef __linux__
void PySimulation::simThreadFunctionRT(PySimulation *pySim) {
	bool notDone = true;
	char timebuf[8];
	int timerfd;
	struct itimerspec ts;
	uint64_t overrun;
	std::unique_lock<std::mutex> lk(*pySim->mut, std::defer_lock);

	pySim->numStep = 0;
	// RT method is limited to timerfd right now; to implement it with Exceptions,
	// look at Simulation::runRT
	timerfd = timerfd_create(CLOCK_REALTIME, 0);
	// TODO: better error mechanism (somehow pass an Exception to the Python thread?)
	if (timerfd < 0) {
		std::perror("Failed to create timerfd");
		std::exit(1);
	}
	ts.it_value.tv_sec = (time_t) pySim->sim->getTimeStep();
	ts.it_value.tv_nsec = (long) (pySim->sim->getTimeStep() *1e9);
	ts.it_interval = ts.it_value;
	// optional start synchronization
	if (pySim->startSync) {
		pySim->sim->step(*pySim->log, *pySim->llog, *pySim->rlog, false); // first step, sending the initial values
		pySim->sim->step(*pySim->log, *pySim->llog, *pySim->rlog, true); // blocking step for synchronization + receiving the initial state of the other network
		pySim->sim->increaseByTimeStep();
	}
	if (timerfd_settime(timerfd, 0, &ts, 0) < 0) {
		std::perror("Failed to arm timerfd");
		std::exit(1);
	}
	while (pySim->running && notDone) {
		notDone = pySim->sim->step(*pySim->log, *pySim->llog, *pySim->rlog);
		if (read(timerfd, timebuf, 8) < 0) {
			std::perror("Read from timerfd failed");
			std::exit(1);
		}
		overrun = *((uint64_t*) timebuf);
		if (overrun > 1) {
			std::cerr << "timerfd overrun of " << overrun-1 << " at " << pySim->sim->getTime() << std::endl;
		}
		pySim->numStep++;
		pySim->sim->increaseByTimeStep();
		// in case it wasn't obvious, pausing a RT simulation is a bad idea
		// as it will most likely lead to overruns, but it's possible nonetheless
		if (pySim->sigPause) {
			lk.lock();
			pySim->state = StatePaused;
			pySim->cond->notify_one();
			pySim->cond->wait(lk);
			pySim->state = StateRunning;
			lk.unlock();
		}
	}
	close(timerfd);
	lk.lock();
	pySim->state = StateDone;
	pySim->cond->notify_one();
}
#endif

PyObject* PySimulation::newfunc(PyTypeObject* type, PyObject *args, PyObject *kwds) {
	PySimulation *self;

	self = (PySimulation*) type->tp_alloc(type, 0);
	if (self) {
		// since mutex, thread etc. have no copy-constructor, but we can't use
		// our own C++ constructor that could be called from python, we need to
		// implement them as pointers
		self->cond = new std::condition_variable();
		self->mut = new std::mutex();
		self->numSwitch = 0;
	}
	return (PyObject*) self;
}

int PySimulation::init(PySimulation* self, PyObject *args, PyObject *kwds) {
	static char *kwlist[] = {"components", "frequency", "timestep", "duration", "log", "llog", "rlog", "rt", "start_sync", NULL};
	double frequency = 50, timestep = 1e-3, duration = DBL_MAX;
	const char *log = nullptr, *llog = nullptr, *rlog = nullptr;

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "O|dddsssbb", kwlist,
		&self->pyComps, &frequency, &timestep, &duration, &log, &llog, &rlog, &self->rt, &self->startSync))
		return -1;
	if (!compsFromPython(self->pyComps, self->comps)) {
		PyErr_SetString(PyExc_TypeError, "Invalid components argument (must by list of dpsim.Component)");
		return -1;
	}
#ifndef __linux__
	if (self->rt) {
		PyErr_SetString(PyExc_SystemError, "RT mode not available on this platform");
		return -1;
	}
#endif
	if (self->startSync && !self->rt) {
		PyErr_Format(PyExc_ValueError, "start_sync only valid in rt mode");
		return -1;
	}
	Py_INCREF(self->pyComps);
	if (log)
		self->log = new Logger(log);
	else
		self->log = new Logger();
	if (rlog)
		self->rlog = new Logger(rlog);
	else
		self->rlog = new Logger();
	if (llog)
		self->llog = new Logger(llog);
	else
		self->llog = new Logger();
	self->sim = new Simulation(self->comps, 2*PI*frequency, timestep, duration, *self->log);
	return 0;
};

void PySimulation::dealloc(PySimulation* self) {
	if (self->simThread) {
		// We have to cancel the running thread here, because otherwise self can't
		// be freed.
		PySimulation::stop((PyObject*)self, NULL);
		self->simThread->join();
		delete self->simThread;
	}

	if (self->sim)
		delete self->sim;
	if (self->log)
		delete self->log;
	if (self->llog)
		delete self->llog;
	if (self->rlog)
		delete self->rlog;
	delete self->mut;
	delete self->cond;

	for (auto it : self->refs) {
		Py_DECREF(it);
	}
	// Since this is not a C++ destructor which would automatically call the
	// destructor of its members, we have to manually call the destructor of
	// the vectors here to free the associated memory.
	self->comps.~vector<BaseComponent*>();
	self->refs.~vector<PyObject*>();

	Py_XDECREF(self->pyComps);
	Py_TYPE(self)->tp_free((PyObject*)self);
}


const char* pyDocSimulationAddInterface =
"add_interface(intf)\n"
"Add an external interface to the simulation. "
"Before each timestep, values are read from this interface and results are written to this interface afterwards. "
"See the documentation of `Interface` for more details.\n"
"\n"
":param intf: The `Interface` to be added.";
PyObject* PySimulation::addInterface(PyObject* self, PyObject* args) {
	PySimulation *pySim = (PySimulation*) self;
	PyObject* pyObj;
	PyInterface* pyIntf;

	if (!PyArg_ParseTuple(args, "O", &pyObj))
		return nullptr;
	if (!PyObject_TypeCheck(pyObj, &PyInterfaceType)) {
		PyErr_SetString(PyExc_TypeError, "Argument must be dpsim.Interface");
		return nullptr;
	}
	pyIntf = (PyInterface*) pyObj;
	pySim->sim->addExternalInterface(pyIntf->intf);
	Py_INCREF(pyObj);
	pySim->refs.push_back(pyObj);
	Py_INCREF(Py_None);
	return Py_None;
}

const char* pyDocSimulationLvector =
"lvector()\n"
"Return the left-side vector of the last step as a list of floats.";
PyObject* PySimulation::lvector(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	if (pySim->state == StateRunning) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently running");
		return nullptr;
	}
	Matrix& lvector = pySim->sim->getLeftSideVector();
	PyObject* list = PyList_New(lvector.rows());
	for (int i = 0; i < lvector.rows(); i++)
		PyList_SetItem(list, i, PyFloat_FromDouble(lvector(i, 0)));
	return list;
}

const char* pyDocSimulationPause =
"pause()\n"
"Pause the simulation at the next possible time (usually, after finishing the current timestep).\n"
"\n"
":raises: ``SystemError`` if the simulation is not running.\n";
PyObject* PySimulation::pause(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	if (pySim->state != StateRunning) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	}
	pySim->sigPause = 1;
	pySim->cond->notify_one();
	while (pySim->state == StateRunning)
		pySim->cond->wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}

const char* pyDocSimulationStart =
"start()\n"
"Start the simulation, or resume it if it has been paused. "
"The simulation runs in a separate thread, so this method doesn't wait for the "
"simulation to finish, but returns immediately.\n"
"\n"
":raises: ``SystemError`` if the simulation is already running or finished.";
PyObject* PySimulation::start(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	if (pySim->state == StateRunning) {
		PyErr_SetString(PyExc_SystemError, "Simulation already started");
		return nullptr;
	} else if (pySim->state == StateDone) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	} else if (pySim->state == StatePaused) {
		pySim->sigPause = 0;
		pySim->cond->notify_one();
	} else {
		pySim->sigPause = 0;
		pySim->state = StateRunning;
		pySim->running = true;
		pySim->simThread = new std::thread(simThreadFunction, pySim);
	}
	Py_INCREF(Py_None);
	return Py_None;
}

const char* pyDocSimulationStep =
"step()\n"
"Perform a single step of the simulation (possibly the first).\n"
"\n"
":raises: ``SystemError`` if the simulation is already running or finished.";
PyObject* PySimulation::step(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	int oldStep = pySim->numStep;
	if (pySim->state == StateStopped) {
		pySim->state = StateRunning;
		pySim->sigPause = 1;
		pySim->running = true;
		pySim->simThread = new std::thread(simThreadFunction, pySim);
	} else if (pySim->state == StatePaused) {
		pySim->sigPause = 1;
		pySim->cond->notify_one();
	} else if (pySim->state == StateDone) {
		PyErr_SetString(PyExc_SystemError, "Simulation already finished");
		return nullptr;
	} else {
		PyErr_SetString(PyExc_SystemError, "Simulation currently running");
		return nullptr;
	}
	while (pySim->numStep == oldStep)
		pySim->cond->wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}

const char* pyDocSimulationStop = 
"stop()\n"
"Stop the simulation at the next possible time. The simulation thread is canceled "
"and the simulation can not be restarted. No-op if the simulation is not running.";
PyObject* PySimulation::stop(PyObject *self, PyObject *args) {
	PySimulation* pySim = (PySimulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	pySim->running = false;
	while (pySim->state == StateRunning)
		pySim->cond->wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}

const char* pyDocSimulationUpdateMatrix =
"update_matrix()\n"
"Recompute the internal system matrix. Must be called after component parameters "
"have been changed during a simulation.";
PyObject* PySimulation::updateMatrix(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	// TODO: this is a quick-and-dirty method that keeps the old matrix in
	// memory
	pySim->sim->addSystemTopology(pySim->comps);
	pySim->sim->switchSystemMatrix(++pySim->numSwitch);
	Py_INCREF(Py_None);
	return Py_None;
}

const char* pyDocSimulationWait = 
"wait()\n"
"Block until the simulation is finished, returning immediately if this is already the case.\n"
"\n"
":raises: ``SystemError`` if the simulation is paused or was not started yet.";
PyObject* PySimulation::wait(PyObject *self, PyObject *args) {
	PySimulation *pySim = (PySimulation*) self;
	std::unique_lock<std::mutex> lk(*pySim->mut);
	if (pySim->state == StateDone) {
		Py_INCREF(Py_None);
		return Py_None;
	} else if (pySim->state == StateStopped) {
		PyErr_SetString(PyExc_SystemError, "Simulation not currently running");
		return nullptr;
	} else if (pySim->state == StatePaused) {
		PyErr_SetString(PyExc_SystemError, "Simulation currently paused");
		return nullptr;
	}
	while (pySim->state == StateRunning)
		pySim->cond->wait(lk);
	Py_INCREF(Py_None);
	return Py_None;
}

static PyMethodDef PySimulation_methods[] = {
	{"add_interface", PySimulation::addInterface, METH_VARARGS, pyDocSimulationAddInterface},
	{"lvector", PySimulation::lvector, METH_NOARGS, pyDocSimulationLvector},
	{"pause", PySimulation::pause, METH_NOARGS, pyDocSimulationPause},
	{"start", PySimulation::start, METH_NOARGS, pyDocSimulationStart},
	{"step", PySimulation::step, METH_NOARGS, pyDocSimulationStep},
	{"stop", PySimulation::stop, METH_NOARGS, pyDocSimulationStop},
	{"update_matrix", PySimulation::updateMatrix, METH_NOARGS, pyDocSimulationUpdateMatrix},
	{"wait", PySimulation::wait, METH_NOARGS, pyDocSimulationWait},
	{NULL, NULL, 0, NULL}
};

const char* pyDocSimulation =
"A single simulation.\n"
"\n"
"Proper ``__init__`` signature:\n"
"\n"
"``__init__(self, components, frequency=50.0, timestep=1e-3, duration=sys.float_info.max, "
"log=None, llog=None, rlog=None, rt=false, start_sync=false)``.\n\n"
"``components`` must be a list of `Component` that are to be simulated.\n\n"
"``frequency`` is the nominal system frequency in Hz.\n\n"
"``timestep`` is the simulation timestep in seconds.\n\n"
"``duration`` is the duration after which the simulation stops; the default value "
"lets the simulation run indefinitely until being stopped manually by `stop`.\n\n"
"``log``, ``llog`` and ``rlog`` are three filenames for log files where "
"general simulation information, the left-side vector and the right-side vector "
"for each timestep are logged, respectively.\n\n"
"If ``rt`` is True, the simulation will run in realtime mode. The simulation will "
"try to match simulation time with the wall clock time; violations will be logged.\n\n"
"If ``start_sync`` is given as well, a specific method for synchronizing the "
"simulation start with other external simulators will be used. After performing "
"a first step with the initial values, the simulation will wait until receiving "
"the first message(s) from the external interface(s) until the realtime simulation "
"starts properly.";
PyTypeObject DPsim::PySimulationType = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"dpsim.Simulation",                /* tp_name */
	sizeof(PySimulation),              /* tp_basicsize */
	0,                                 /* tp_itemsize */
	(destructor)PySimulation::dealloc, /* tp_dealloc */
	0,                                 /* tp_print */
	0,                                 /* tp_getattr */
	0,                                 /* tp_setattr */
	0,                                 /* tp_reserved */
	0,                                 /* tp_repr */
	0,                                 /* tp_as_number */
	0,                                 /* tp_as_sequence */
	0,                                 /* tp_as_mapping */
	0,                                 /* tp_hash  */
	0,                                 /* tp_call */
	0,                                 /* tp_str */
	0,                                 /* tp_getattro */
	0,                                 /* tp_setattro */
	0,                                 /* tp_as_buffer */
	Py_TPFLAGS_DEFAULT |
		Py_TPFLAGS_BASETYPE,           /* tp_flags */
	pyDocSimulation,                   /* tp_doc */
	0,                                 /* tp_traverse */
	0,                                 /* tp_clear */
	0,                                 /* tp_richcompare */
	0,                                 /* tp_weaklistoffset */
	0,                                 /* tp_iter */
	0,                                 /* tp_iternext */
	PySimulation_methods,              /* tp_methods */
	0,                                 /* tp_members */
	0,                                 /* tp_getset */
	0,                                 /* tp_base */
	0,                                 /* tp_dict */
	0,                                 /* tp_descr_get */
	0,                                 /* tp_descr_set */
	0,                                 /* tp_dictoffset */
	(initproc)PySimulation::init,      /* tp_init */
	0,                                 /* tp_alloc */
	PySimulation::newfunc,             /* tp_new */
};
